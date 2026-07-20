#ifndef PYRAMID_DATA_MODEL_AGRA_CABI_H
#define PYRAMID_DATA_MODEL_AGRA_CABI_H

#include <stdint.h>
#include "pyramid_datamodel_cabi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pyramid_data_model_agra_SCI_ControlsChoiceType_c {
  uint8_t has_standard_compartment;
  int32_t standard_compartment;
  uint8_t has_sub_compartment;
  pyramid_str_t sub_compartment;
} pyramid_data_model_agra_SCI_ControlsChoiceType_c;

typedef struct pyramid_data_model_agra_OwnerProducerChoiceType_c {
  uint8_t has_government_identifier;
  int32_t government_identifier;
  uint8_t has_nato_special_word;
  pyramid_str_t nato_special_word;
} pyramid_data_model_agra_OwnerProducerChoiceType_c;

typedef struct pyramid_data_model_agra_AtomicEnergyMarkingsChoiceType_c {
  uint8_t has_markings;
  int32_t markings;
  uint8_t has_sigma_markings;
  int32_t sigma_markings;
} pyramid_data_model_agra_AtomicEnergyMarkingsChoiceType_c;

typedef struct pyramid_data_model_agra_NonIC_MarkingsChoiceType_c {
  uint8_t has_standard_marking;
  int32_t standard_marking;
  uint8_t has_alternate_marking;
  pyramid_str_t alternate_marking;
} pyramid_data_model_agra_NonIC_MarkingsChoiceType_c;

typedef struct pyramid_data_model_agra_FGI_SourceOpenChoiceType_c {
  uint8_t has_foreign_government_identifier;
  int32_t foreign_government_identifier;
  uint8_t has_nato_special_word;
  pyramid_str_t nato_special_word;
} pyramid_data_model_agra_FGI_SourceOpenChoiceType_c;

typedef struct pyramid_data_model_agra_ReleasableToChoiceType_c {
  uint8_t has_government_identifier;
  int32_t government_identifier;
  uint8_t has_nato_special_word;
  pyramid_str_t nato_special_word;
} pyramid_data_model_agra_ReleasableToChoiceType_c;

typedef struct pyramid_data_model_agra_SecurityInformationType_c {
  int32_t classification;
  pyramid_slice_t owner_producer;
  uint8_t has_joint;
  uint8_t joint;
  pyramid_slice_t sci_controls;
  pyramid_slice_t sar_identifier;
  pyramid_slice_t atomic_energy_markings;
  pyramid_slice_t dissemination_controls;
  pyramid_slice_t display_only_to;
  pyramid_slice_t fgi_source_open;
  pyramid_slice_t releasable_to;
  pyramid_slice_t non_ic_markings;
  pyramid_str_t classified_by;
  pyramid_str_t compilation_reason;
  pyramid_str_t derivatively_classified_by;
  pyramid_str_t classification_reason;
  pyramid_slice_t non_us_controls;
  pyramid_str_t derived_from;
  pyramid_str_t declass_date;
  pyramid_str_t declass_event;
  pyramid_slice_t declass_exception;
} pyramid_data_model_agra_SecurityInformationType_c;

typedef struct pyramid_data_model_agra_ID_Type_c {
  pyramid_str_t uuid;
  pyramid_str_t descriptive_label;
} pyramid_data_model_agra_ID_Type_c;

typedef struct pyramid_data_model_agra_SystemID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SystemID_Type_c;

typedef struct pyramid_data_model_agra_VersionedID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
  uint8_t has_version;
  uint32_t version;
} pyramid_data_model_agra_VersionedID_Type_c;

typedef struct pyramid_data_model_agra_MissionID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_MissionID_Type_c;

typedef struct pyramid_data_model_agra_ServiceID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
  pyramid_str_t service_version;
} pyramid_data_model_agra_ServiceID_Type_c;

typedef struct pyramid_data_model_agra_HeaderType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  pyramid_str_t timestamp;
  pyramid_str_t schema_version;
  int32_t mode;
  uint8_t has_service_id;
  pyramid_data_model_agra_ServiceID_Type_c service_id;
  uint8_t has_mission_id;
  pyramid_data_model_agra_MissionID_Type_c mission_id;
} pyramid_data_model_agra_HeaderType_c;

typedef struct pyramid_data_model_agra_MessageType_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;
  pyramid_data_model_agra_HeaderType_c message_header;
} pyramid_data_model_agra_MessageType_c;

typedef struct pyramid_data_model_agra_MA_RequirementTaxonomyType_c {
  pyramid_slice_t effect;
  pyramid_slice_t action;
  pyramid_slice_t task;
  pyramid_slice_t capability_command;
  pyramid_slice_t response;
} pyramid_data_model_agra_MA_RequirementTaxonomyType_c;

typedef struct pyramid_data_model_agra_ForeignKeyType_c {
  pyramid_str_t key;
  pyramid_str_t system_name;
} pyramid_data_model_agra_ForeignKeyType_c;

typedef struct pyramid_data_model_agra_SpecificEmitterIdentityType_c {
  pyramid_data_model_agra_ForeignKeyType_c specific_emitter_key;
} pyramid_data_model_agra_SpecificEmitterIdentityType_c;

typedef struct pyramid_data_model_agra_CountryCodeType_c {
  uint8_t has_country_name;
  int32_t country_name;
  uint8_t has_operator_unique_asset_name;
  int32_t operator_unique_asset_name;
} pyramid_data_model_agra_CountryCodeType_c;

typedef struct pyramid_data_model_agra_StandardIdentityType_c {
  int32_t standard_identity;
  uint8_t has_allegiance;
  pyramid_data_model_agra_CountryCodeType_c allegiance;
} pyramid_data_model_agra_StandardIdentityType_c;

typedef struct pyramid_data_model_agra_SpecificIdentityType_c {
  uint32_t specific_type;
  int32_t specific_type_category;
  uint8_t has_site_type;
  int32_t site_type;
  pyramid_str_t specific_type_model;
} pyramid_data_model_agra_SpecificIdentityType_c;

typedef struct pyramid_data_model_agra_FacilityIdentificationType_c {
  uint8_t has_site_identifier;
  uint32_t site_identifier;
  uint8_t has_sensor_identifier;
  uint32_t sensor_identifier;
  uint8_t has_foreign_facility_key;
  pyramid_data_model_agra_ForeignKeyType_c foreign_facility_key;
} pyramid_data_model_agra_FacilityIdentificationType_c;

typedef struct pyramid_data_model_agra_IFF_Mode5Type_c {
  int32_t national_origin;
  int32_t pin;
  uint8_t has_mode5_indicator;
  int32_t mode5_indicator;
  pyramid_str_t mode1_code;
} pyramid_data_model_agra_IFF_Mode5Type_c;

typedef struct pyramid_data_model_agra_IFF_ModeS_Type_c {
  uint32_t icao;
  pyramid_str_t aircraft_identifier;
  uint8_t enabled;
} pyramid_data_model_agra_IFF_ModeS_Type_c;

typedef struct pyramid_data_model_agra_IFF_OctalModeType_c {
  pyramid_str_t code;
  uint8_t enabled;
} pyramid_data_model_agra_IFF_OctalModeType_c;

typedef struct pyramid_data_model_agra_IFF_Mode4Type_c {
  uint8_t has_mode4_indicator;
  int32_t mode4_indicator;
  uint8_t has_mode4_code;
  int32_t mode4_code;
} pyramid_data_model_agra_IFF_Mode4Type_c;

typedef struct pyramid_data_model_agra_IFF_Mode1Type_c {
  pyramid_str_t code;
  uint8_t enabled;
} pyramid_data_model_agra_IFF_Mode1Type_c;

typedef struct pyramid_data_model_agra_IFF_Type_c {
  uint8_t has_mode1;
  pyramid_data_model_agra_IFF_Mode1Type_c mode1;
  uint8_t has_mode2;
  pyramid_data_model_agra_IFF_OctalModeType_c mode2;
  pyramid_slice_t mode3_a;
  uint8_t has_mode3_lc;
  pyramid_data_model_agra_IFF_OctalModeType_c mode3_lc;
  uint8_t has_mode4;
  pyramid_data_model_agra_IFF_Mode4Type_c mode4;
  uint8_t has_mode5;
  pyramid_data_model_agra_IFF_Mode5Type_c mode5;
  uint8_t has_mode_s;
  pyramid_data_model_agra_IFF_ModeS_Type_c mode_s;
  uint8_t has_mode_c;
  pyramid_data_model_agra_IFF_OctalModeType_c mode_c;
} pyramid_data_model_agra_IFF_Type_c;

typedef struct pyramid_data_model_agra_DataLinkIdentifierPET_c {
} pyramid_data_model_agra_DataLinkIdentifierPET_c;

typedef struct pyramid_data_model_agra_InternationalDesignatorType_c {
  uint32_t year;
  uint32_t number;
  pyramid_str_t piece;
} pyramid_data_model_agra_InternationalDesignatorType_c;

typedef struct pyramid_data_model_agra_MissionInformationType_c {
  pyramid_str_t mission_name;
  pyramid_str_t mission_description;
} pyramid_data_model_agra_MissionInformationType_c;

typedef struct pyramid_data_model_agra_SatelliteIdentifierType_c {
  uint32_t satellite_number;
  pyramid_str_t satellite_name;
  uint8_t has_international_designator;
  pyramid_data_model_agra_InternationalDesignatorType_c international_designator;
  pyramid_slice_t mission_information;
} pyramid_data_model_agra_SatelliteIdentifierType_c;

typedef struct pyramid_data_model_agra_AIS_Type_c {
  pyramid_str_t mmsi_number;
  pyramid_str_t imo_number;
  pyramid_str_t vessel_name;
  pyramid_str_t call_sign;
} pyramid_data_model_agra_AIS_Type_c;

typedef struct pyramid_data_model_agra_VehicleUniqueIdentifierType_c {
  uint8_t has_ais;
  pyramid_data_model_agra_AIS_Type_c ais;
  uint8_t has_tail_number;
  pyramid_str_t tail_number;
  uint8_t has_satellite;
  pyramid_data_model_agra_SatelliteIdentifierType_c satellite;
  uint8_t has_alternate_identifier;
  pyramid_str_t alternate_identifier;
} pyramid_data_model_agra_VehicleUniqueIdentifierType_c;

typedef struct pyramid_data_model_agra_CallSignType_c {
  pyramid_str_t key;  /* from ForeignKeyType */
  pyramid_str_t system_name;  /* from ForeignKeyType */
} pyramid_data_model_agra_CallSignType_c;

typedef struct pyramid_data_model_agra_VehicleIdentificationType_c {
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_vehicle_unique_identifier;
  pyramid_data_model_agra_VehicleUniqueIdentifierType_c vehicle_unique_identifier;
  uint8_t has_iff;
  pyramid_data_model_agra_IFF_Type_c iff;
  pyramid_slice_t call_sign;
  pyramid_slice_t radar_cross_section;
  pyramid_slice_t data_link_identifier;
} pyramid_data_model_agra_VehicleIdentificationType_c;

typedef struct pyramid_data_model_agra_CommunicationsEmitterIdentityType_c {
  pyramid_str_t cenot_identifier;
  uint8_t has_comm_mode;
  int32_t comm_mode;
  uint8_t has_emitter_index;
  pyramid_data_model_agra_ForeignKeyType_c emitter_index;
} pyramid_data_model_agra_CommunicationsEmitterIdentityType_c;

typedef struct pyramid_data_model_agra_MissileEmitterIdentityType_c {
  pyramid_str_t elnot_identifier;
  uint8_t has_missile_mode;
  int32_t missile_mode;
  uint8_t has_emitter_index;
  pyramid_data_model_agra_ForeignKeyType_c emitter_index;
} pyramid_data_model_agra_MissileEmitterIdentityType_c;

typedef struct pyramid_data_model_agra_RadarEmitterIdentityType_c {
  pyramid_str_t elnot_identifier;
  uint8_t has_dieqp_key;
  pyramid_data_model_agra_ForeignKeyType_c dieqp_key;
  uint8_t has_radar_mode;
  int32_t radar_mode;
  uint8_t has_track_mode;
  int32_t track_mode;
  uint8_t has_emitter_index;
  pyramid_data_model_agra_ForeignKeyType_c emitter_index;
} pyramid_data_model_agra_RadarEmitterIdentityType_c;

typedef struct pyramid_data_model_agra_JammerEmitterIdentityType_c {
  pyramid_str_t elnot_identifier;
  uint8_t has_received_power;
  double received_power;
  uint8_t has_jam_mode;
  int32_t jam_mode;
  uint8_t has_emitter_index;
  pyramid_data_model_agra_ForeignKeyType_c emitter_index;
} pyramid_data_model_agra_JammerEmitterIdentityType_c;

typedef struct pyramid_data_model_agra_EmitterIdentityCategoryType_c {
  uint8_t has_radar;
  pyramid_data_model_agra_RadarEmitterIdentityType_c radar;
  uint8_t has_communications;
  pyramid_data_model_agra_CommunicationsEmitterIdentityType_c communications;
  uint8_t has_jammer;
  pyramid_data_model_agra_JammerEmitterIdentityType_c jammer;
  uint8_t has_missile;
  pyramid_data_model_agra_MissileEmitterIdentityType_c missile;
} pyramid_data_model_agra_EmitterIdentityCategoryType_c;

typedef struct pyramid_data_model_agra_EmitterIdentityType_c {
  pyramid_data_model_agra_EmitterIdentityCategoryType_c emitter_category;
  pyramid_str_t nato_spot_number;
} pyramid_data_model_agra_EmitterIdentityType_c;

typedef struct pyramid_data_model_agra_OrbitRegimeType_c {
  int32_t orbit_regime;
  pyramid_slice_t orbit_class;
} pyramid_data_model_agra_OrbitRegimeType_c;

typedef struct pyramid_data_model_agra_EnvironmentIdentityType_c {
  int32_t environment;
  uint8_t has_orbit_regime;
  pyramid_data_model_agra_OrbitRegimeType_c orbit_regime;
} pyramid_data_model_agra_EnvironmentIdentityType_c;

typedef struct pyramid_data_model_agra_StoreType_c {
  uint32_t store_type;
  int32_t store_category;
  uint8_t has_store_type_variant;
  pyramid_data_model_agra_ForeignKeyType_c store_type_variant;
} pyramid_data_model_agra_StoreType_c;

typedef struct pyramid_data_model_agra_PlatformIdentityType_c {
  uint32_t platform_type;
  int32_t platform_type_category;
  uint8_t has_threat_type;
  int32_t threat_type;
  uint8_t has_unit_type;
  int32_t unit_type;
  uint8_t has_launch_capability;
  int32_t launch_capability;
  uint8_t has_submarine_confidence_level;
  int32_t submarine_confidence_level;
} pyramid_data_model_agra_PlatformIdentityType_c;

typedef struct pyramid_data_model_agra_EOB_SitePIN_Type_c {
  pyramid_str_t key;  /* from ForeignKeyType */
  pyramid_str_t system_name;  /* from ForeignKeyType */
} pyramid_data_model_agra_EOB_SitePIN_Type_c;

typedef struct pyramid_data_model_agra_EOB_IdentityBaseType_c {
  uint8_t has_site_pin;
  pyramid_data_model_agra_EOB_SitePIN_Type_c site_pin;
  pyramid_str_t site_name;
  uint8_t has_internal_id;
  pyramid_data_model_agra_ID_Type_c internal_id;
} pyramid_data_model_agra_EOB_IdentityBaseType_c;

typedef struct pyramid_data_model_agra_EOB_EquipmentType_c {
  pyramid_str_t key;  /* from ForeignKeyType */
  pyramid_str_t system_name;  /* from ForeignKeyType */
} pyramid_data_model_agra_EOB_EquipmentType_c;

typedef struct pyramid_data_model_agra_EOB_EquipmentIdentityType_c {
  uint8_t has_site_pin;
  pyramid_data_model_agra_EOB_SitePIN_Type_c site_pin;  /* from EOB_IdentityBaseType */
  pyramid_str_t site_name;  /* from EOB_IdentityBaseType */
  uint8_t has_internal_id;
  pyramid_data_model_agra_ID_Type_c internal_id;  /* from EOB_IdentityBaseType */
  uint8_t has_equipment_identifier_number;
  pyramid_data_model_agra_EOB_EquipmentType_c equipment_identifier_number;
} pyramid_data_model_agra_EOB_EquipmentIdentityType_c;

typedef struct pyramid_data_model_agra_BasicEncyclopediaNumberType_c {
  pyramid_str_t world_area_code;
  pyramid_str_t record_originator;
  pyramid_str_t one_up_number;
} pyramid_data_model_agra_BasicEncyclopediaNumberType_c;

typedef struct pyramid_data_model_agra_EOB_SiteIdentityType_c {
  uint8_t has_site_pin;
  pyramid_data_model_agra_EOB_SitePIN_Type_c site_pin;  /* from EOB_IdentityBaseType */
  pyramid_str_t site_name;  /* from EOB_IdentityBaseType */
  uint8_t has_internal_id;
  pyramid_data_model_agra_ID_Type_c internal_id;  /* from EOB_IdentityBaseType */
  uint8_t has_be_number;
  pyramid_data_model_agra_BasicEncyclopediaNumberType_c be_number;
  pyramid_str_t o_suffix;
  pyramid_str_t category;
} pyramid_data_model_agra_EOB_SiteIdentityType_c;

typedef struct pyramid_data_model_agra_EOB_IdentityType_c {
  uint8_t has_site;
  pyramid_data_model_agra_EOB_SiteIdentityType_c site;
  uint8_t has_equipment;
  pyramid_data_model_agra_EOB_EquipmentIdentityType_c equipment;
} pyramid_data_model_agra_EOB_IdentityType_c;

typedef struct pyramid_data_model_agra_IdentityType_c {
  uint8_t has_standard;
  pyramid_data_model_agra_StandardIdentityType_c standard;
  uint8_t has_environment;
  pyramid_data_model_agra_EnvironmentIdentityType_c environment;
  uint8_t has_platform;
  pyramid_data_model_agra_PlatformIdentityType_c platform;
  uint8_t has_specific;
  pyramid_data_model_agra_SpecificIdentityType_c specific;
  pyramid_slice_t emitter;
  pyramid_slice_t specific_emitter;
  uint8_t has_specific_facility;
  pyramid_data_model_agra_FacilityIdentificationType_c specific_facility;
  uint8_t has_specific_vehicle;
  pyramid_data_model_agra_VehicleIdentificationType_c specific_vehicle;
  uint8_t has_eob;
  pyramid_data_model_agra_EOB_IdentityType_c eob;
  uint8_t has_weapon;
  pyramid_data_model_agra_StoreType_c weapon;
} pyramid_data_model_agra_IdentityType_c;

typedef struct pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c;

typedef struct pyramid_data_model_agra_PackageID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_PackageID_Type_c;

typedef struct pyramid_data_model_agra_MA_AllocationChoiceType_c {
  uint8_t has_system_id;
  pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c system_id;
  uint8_t has_package_id;
  pyramid_data_model_agra_PackageID_Type_c package_id;
} pyramid_data_model_agra_MA_AllocationChoiceType_c;

typedef struct pyramid_data_model_agra_CapabilityID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_CapabilityID_Type_c;

typedef struct pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c {
  pyramid_data_model_agra_MA_AllocationChoiceType_c allocation;
  pyramid_slice_t identity;
  pyramid_slice_t capability_id;
} pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c;

typedef struct pyramid_data_model_agra_MA_RequirementAllocationParametersType_c {
  uint8_t has_required_allocation;
  pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c required_allocation;
  uint8_t has_preferred_allocation;
  pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c preferred_allocation;
} pyramid_data_model_agra_MA_RequirementAllocationParametersType_c;

typedef struct pyramid_data_model_agra_DataProductClassificationLevelType_c {
  pyramid_data_model_agra_SecurityInformationType_c min;
  uint8_t has_max;
  pyramid_data_model_agra_SecurityInformationType_c max;
} pyramid_data_model_agra_DataProductClassificationLevelType_c;

typedef struct pyramid_data_model_agra_RequirementDependencyBaseType_c {
  int32_t dependency_type;
  int32_t dependency_extent;
  pyramid_str_t earliest_time;
  pyramid_str_t latest_time;
} pyramid_data_model_agra_RequirementDependencyBaseType_c;

typedef struct pyramid_data_model_agra_CommandID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_CommandID_Type_c;

typedef struct pyramid_data_model_agra_ActionID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_ActionID_Type_c;

typedef struct pyramid_data_model_agra_EffectID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_EffectID_Type_c;

typedef struct pyramid_data_model_agra_TaskID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_TaskID_Type_c;

typedef struct pyramid_data_model_agra_ResponseID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
  pyramid_slice_t option_index;
} pyramid_data_model_agra_ResponseID_Type_c;

typedef struct pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c {
  uint8_t has_effect_id;
  pyramid_data_model_agra_EffectID_Type_c effect_id;
  uint8_t has_action_id;
  pyramid_data_model_agra_ActionID_Type_c action_id;
  uint8_t has_task_id;
  pyramid_data_model_agra_TaskID_Type_c task_id;
  uint8_t has_capability_command_id;
  pyramid_data_model_agra_CommandID_Type_c capability_command_id;
  uint8_t has_response_id;
  pyramid_data_model_agra_ResponseID_Type_c response_id;
} pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c;

typedef struct pyramid_data_model_agra_RequirementDependencyType_c {
  int32_t dependency_type;  /* from RequirementDependencyBaseType */
  int32_t dependency_extent;  /* from RequirementDependencyBaseType */
  pyramid_str_t earliest_time;  /* from RequirementDependencyBaseType */
  pyramid_str_t latest_time;  /* from RequirementDependencyBaseType */
  pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c reference_requirement;
} pyramid_data_model_agra_RequirementDependencyType_c;

typedef struct pyramid_data_model_agra_DateTimeRangeType_c {
  pyramid_str_t begin;
  pyramid_str_t end;
} pyramid_data_model_agra_DateTimeRangeType_c;

typedef struct pyramid_data_model_agra_TimeWindowType_c {
  uint8_t has_range;
  pyramid_data_model_agra_DateTimeRangeType_c range;
  pyramid_str_t duration;
} pyramid_data_model_agra_TimeWindowType_c;

typedef struct pyramid_data_model_agra_WeekdayIntervalType_c {
  pyramid_data_model_agra_DateTimeRangeType_c time_span;
  pyramid_slice_t weekday;
  uint32_t weekly_interval_period;
  pyramid_str_t start_time;
  pyramid_str_t duration;
} pyramid_data_model_agra_WeekdayIntervalType_c;

typedef struct pyramid_data_model_agra_LOS_Type_c {
  uint8_t has_slant_range;
  double slant_range;
  uint8_t has_bearing;
  double bearing;
  uint8_t has_elevation;
  double elevation;
} pyramid_data_model_agra_LOS_Type_c;

typedef struct pyramid_data_model_agra_TimeErrorType_c {
  pyramid_str_t early;
  pyramid_str_t late;
} pyramid_data_model_agra_TimeErrorType_c;

typedef struct pyramid_data_model_agra_RTN_PositionDeltaType_c {
  uint8_t has_radial_delta;
  double radial_delta;
  uint8_t has_transverse_delta;
  double transverse_delta;
  uint8_t has_normal_delta;
  double normal_delta;
} pyramid_data_model_agra_RTN_PositionDeltaType_c;

typedef struct pyramid_data_model_agra_RTN_VelocityDeltaType_c {
  uint8_t has_radial_delta;
  double radial_delta;
  uint8_t has_transverse_delta;
  double transverse_delta;
  uint8_t has_normal_delta;
  double normal_delta;
} pyramid_data_model_agra_RTN_VelocityDeltaType_c;

typedef struct pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c {
  uint8_t has_position_error_threshold;
  pyramid_data_model_agra_RTN_PositionDeltaType_c position_error_threshold;
  uint8_t has_velocity_error_threshold;
  pyramid_data_model_agra_RTN_VelocityDeltaType_c velocity_error_threshold;
  uint8_t has_time_threshold;
  pyramid_data_model_agra_TimeErrorType_c time_threshold;
} pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c;

typedef struct pyramid_data_model_agra_LOS_InertialA_Type_c {
  uint8_t has_azimuth;
  double azimuth;
  uint8_t has_elevation;
  double elevation;
  uint8_t has_slant_range;
  double slant_range;
} pyramid_data_model_agra_LOS_InertialA_Type_c;

typedef struct pyramid_data_model_agra_RepetitionPositionChangeType_c {
  uint8_t has_los_bearing_elevation;
  pyramid_data_model_agra_LOS_Type_c los_bearing_elevation;
  uint8_t has_los_az_el;
  pyramid_data_model_agra_LOS_InertialA_Type_c los_az_el;
  uint8_t has_orbital_rtn;
  pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c orbital_rtn;
} pyramid_data_model_agra_RepetitionPositionChangeType_c;

typedef struct pyramid_data_model_agra_RepetitionEventType_c {
  uint8_t has_position_change;
  pyramid_data_model_agra_RepetitionPositionChangeType_c position_change;
  uint8_t has_route_event;
  int32_t route_event;
  uint8_t has_orbital_event;
  int32_t orbital_event;
} pyramid_data_model_agra_RepetitionEventType_c;

typedef struct pyramid_data_model_agra_LOS_InertialB_Type_c {
  double azimuth;
  double elevation;
  uint8_t has_slant_range;
  double slant_range;
} pyramid_data_model_agra_LOS_InertialB_Type_c;

typedef struct pyramid_data_model_agra_EventOffsetChoiceType_c {
  uint8_t has_offset_time;
  pyramid_str_t offset_time;
  uint8_t has_offset_angle;
  double offset_angle;
  uint8_t has_az_el;
  pyramid_data_model_agra_LOS_InertialB_Type_c az_el;
} pyramid_data_model_agra_EventOffsetChoiceType_c;

typedef struct pyramid_data_model_agra_COE_PositionType_c {
  uint8_t has_mean_anomaly;
  double mean_anomaly;
  uint8_t has_argument_of_latitude;
  double argument_of_latitude;
  uint8_t has_true_longitude;
  double true_longitude;
} pyramid_data_model_agra_COE_PositionType_c;

typedef struct pyramid_data_model_agra_COE_NonEquatorialOrientationType_c {
  double right_ascension_of_ascending_node;
  double argument_of_perigee;
} pyramid_data_model_agra_COE_NonEquatorialOrientationType_c;

typedef struct pyramid_data_model_agra_COE_EquatorialOrientationType_c {
  uint8_t has_longitude_of_perigee;
  double longitude_of_perigee;
} pyramid_data_model_agra_COE_EquatorialOrientationType_c;

typedef struct pyramid_data_model_agra_COE_OrientationType_c {
  uint8_t has_non_equatorial_orbit;
  pyramid_data_model_agra_COE_NonEquatorialOrientationType_c non_equatorial_orbit;
  uint8_t has_equatorial_orbit;
  pyramid_data_model_agra_COE_EquatorialOrientationType_c equatorial_orbit;
} pyramid_data_model_agra_COE_OrientationType_c;

typedef struct pyramid_data_model_agra_COE_OrbitBaseType_c {
  double eccentricity;
  double inclination;
  pyramid_data_model_agra_COE_OrientationType_c orientation;
} pyramid_data_model_agra_COE_OrbitBaseType_c;

typedef struct pyramid_data_model_agra_TLE_BaseType_c {
  double eccentricity;  /* from COE_OrbitBaseType */
  double inclination;  /* from COE_OrbitBaseType */
  pyramid_data_model_agra_COE_OrientationType_c orientation;  /* from COE_OrbitBaseType */
  pyramid_str_t epoch;
  uint8_t has_element_set_number;
  uint32_t element_set_number;
  uint8_t has_first_time_derivative_of_mean_motion;
  double first_time_derivative_of_mean_motion;
  uint8_t has_second_time_derivative_of_mean_motion;
  double second_time_derivative_of_mean_motion;
  uint8_t has_bstar_drag;
  double bstar_drag;
  pyramid_data_model_agra_COE_PositionType_c position;
  double mean_motion;
  uint8_t has_revolution_number;
  uint32_t revolution_number;
} pyramid_data_model_agra_TLE_BaseType_c;

typedef struct pyramid_data_model_agra_RTN_AccelerationType_c {
  double radial;
  double transverse;
  double normal;
} pyramid_data_model_agra_RTN_AccelerationType_c;

typedef struct pyramid_data_model_agra_QuaternionType_c {
  double qs;
  double qx;
  double qy;
  double qz;
} pyramid_data_model_agra_QuaternionType_c;

typedef struct pyramid_data_model_agra_CovarianceMatrixType_c {
  int32_t category;
  pyramid_slice_t covariance;
} pyramid_data_model_agra_CovarianceMatrixType_c;

typedef struct pyramid_data_model_agra_RTN_PositionType_c {
  double radial;
  double transverse;
  double normal;
} pyramid_data_model_agra_RTN_PositionType_c;

typedef struct pyramid_data_model_agra_RTN_VelocityType_c {
  double radial;
  double transverse;
  double normal;
} pyramid_data_model_agra_RTN_VelocityType_c;

typedef struct pyramid_data_model_agra_RTN_KinematicsType_c {
  pyramid_str_t epoch;
  pyramid_data_model_agra_RTN_PositionType_c position;
  pyramid_data_model_agra_RTN_VelocityType_c velocity;
  uint8_t has_acceleration;
  pyramid_data_model_agra_RTN_AccelerationType_c acceleration;
  uint8_t has_attitude;
  pyramid_data_model_agra_QuaternionType_c attitude;
  uint8_t has_covariance;
  pyramid_data_model_agra_CovarianceMatrixType_c covariance;
} pyramid_data_model_agra_RTN_KinematicsType_c;

typedef struct pyramid_data_model_agra_EntityID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_EntityID_Type_c;

typedef struct pyramid_data_model_agra_AssetType_c {
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_AssetType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c {
  double x;
  double y;
  double z;
} pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsVelocityType_c {
  double x;
  double y;
  double z;
} pyramid_data_model_agra_OrbitalKinematicsVelocityType_c;

typedef struct pyramid_data_model_agra_BCRS_PositionType_c {
  double x;
  double y;
  double z;
} pyramid_data_model_agra_BCRS_PositionType_c;

typedef struct pyramid_data_model_agra_BCRS_KinematicsType_c {
  pyramid_str_t epoch;
  pyramid_data_model_agra_BCRS_PositionType_c position;
  pyramid_data_model_agra_OrbitalKinematicsVelocityType_c velocity;
  uint8_t has_acceleration;
  pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c acceleration;
  uint8_t has_attitude;
  pyramid_data_model_agra_QuaternionType_c attitude;
  uint8_t has_covariance;
  pyramid_data_model_agra_CovarianceMatrixType_c covariance;
} pyramid_data_model_agra_BCRS_KinematicsType_c;

typedef struct pyramid_data_model_agra_GCRS_PositionType_c {
  double x;
  double y;
  double z;
} pyramid_data_model_agra_GCRS_PositionType_c;

typedef struct pyramid_data_model_agra_GCRS_KinematicsType_c {
  pyramid_str_t epoch;
  pyramid_data_model_agra_GCRS_PositionType_c position;
  pyramid_data_model_agra_OrbitalKinematicsVelocityType_c velocity;
  uint8_t has_acceleration;
  pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c acceleration;
  uint8_t has_attitude;
  pyramid_data_model_agra_QuaternionType_c attitude;
  uint8_t has_covariance;
  pyramid_data_model_agra_CovarianceMatrixType_c covariance;
} pyramid_data_model_agra_GCRS_KinematicsType_c;

typedef struct pyramid_data_model_agra_J2K_PositionType_c {
  double x;
  double y;
  double z;
} pyramid_data_model_agra_J2K_PositionType_c;

typedef struct pyramid_data_model_agra_J2K_KinematicsType_c {
  pyramid_str_t epoch;
  pyramid_data_model_agra_J2K_PositionType_c position;
  pyramid_data_model_agra_OrbitalKinematicsVelocityType_c velocity;
  uint8_t has_acceleration;
  pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c acceleration;
  uint8_t has_attitude;
  pyramid_data_model_agra_QuaternionType_c attitude;
  uint8_t has_covariance;
  pyramid_data_model_agra_CovarianceMatrixType_c covariance;
} pyramid_data_model_agra_J2K_KinematicsType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c {
  uint8_t has_j2_k;
  pyramid_data_model_agra_J2K_KinematicsType_c j2_k;
  uint8_t has_gcrs;
  pyramid_data_model_agra_GCRS_KinematicsType_c gcrs;
  uint8_t has_bcrs;
  pyramid_data_model_agra_BCRS_KinematicsType_c bcrs;
} pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c {
  pyramid_data_model_agra_RTN_KinematicsType_c relative_object_kinematics;
  pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c reference_object_kinematics;
  uint8_t has_reference_asset;
  pyramid_data_model_agra_AssetType_c reference_asset;
} pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsChoiceType_c {
  uint8_t has_standard_frame;
  pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c standard_frame;
  uint8_t has_orbiting_object_relative;
  pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c orbiting_object_relative;
} pyramid_data_model_agra_OrbitalKinematicsChoiceType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c {
  uint8_t has_j2_k_state_vector;
  pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c j2_k_state_vector;
  uint8_t has_gcrs_state_vector;
  pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c gcrs_state_vector;
  uint8_t has_bcrs_state_vector;
  pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c bcrs_state_vector;
} pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsRelativeStateVectorType_c {
  pyramid_data_model_agra_RTN_KinematicsType_c relative_object_kinematics;
  pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c reference_object_kinematics;
} pyramid_data_model_agra_OrbitalKinematicsRelativeStateVectorType_c;

typedef struct pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c {
  pyramid_slice_t relative_state_vector;
  uint8_t has_reference_asset;
  pyramid_data_model_agra_AssetType_c reference_asset;
} pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c;

typedef struct pyramid_data_model_agra_OrbitalEphemerisChoiceType_c {
  uint8_t has_standard_frame;
  pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c standard_frame;
  uint8_t has_orbiting_object_relative;
  pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c orbiting_object_relative;
} pyramid_data_model_agra_OrbitalEphemerisChoiceType_c;

typedef struct pyramid_data_model_agra_RTN_PositionSigmaType_c {
  double radial_sigma;
  double transverse_sigma;
  double normal_sigma;
} pyramid_data_model_agra_RTN_PositionSigmaType_c;

typedef struct pyramid_data_model_agra_RTN_VelocitySigmaType_c {
  double radial_sigma;
  double transverse_sigma;
  double normal_sigma;
} pyramid_data_model_agra_RTN_VelocitySigmaType_c;

typedef struct pyramid_data_model_agra_VCM_DataType_c {
  uint8_t has_position_sigma;
  pyramid_data_model_agra_RTN_PositionSigmaType_c position_sigma;
  uint8_t has_velocity_sigma;
  pyramid_data_model_agra_RTN_VelocitySigmaType_c velocity_sigma;
  uint8_t has_weighted_rms;
  double weighted_rms;
  pyramid_data_model_agra_CovarianceMatrixType_c covariance_matrix;
} pyramid_data_model_agra_VCM_DataType_c;

typedef struct pyramid_data_model_agra_OrbitalVCM_Type_c {
  pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c state_vector;
  pyramid_data_model_agra_VCM_DataType_c covariance_data;
  uint8_t has_revolution_number;
  uint32_t revolution_number;
} pyramid_data_model_agra_OrbitalVCM_Type_c;

typedef struct pyramid_data_model_agra_OrbitKinematicsType_c {
  uint8_t has_element_set;
  pyramid_data_model_agra_TLE_BaseType_c element_set;
  uint8_t has_ephemeris;
  pyramid_data_model_agra_OrbitalEphemerisChoiceType_c ephemeris;
  uint8_t has_vcm;
  pyramid_data_model_agra_OrbitalVCM_Type_c vcm;
  uint8_t has_single_vector;
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c single_vector;
} pyramid_data_model_agra_OrbitKinematicsType_c;

typedef struct pyramid_data_model_agra_ReferenceAssetKinematicsType_c {
  pyramid_data_model_agra_AssetType_c reference_asset;
  uint8_t has_orbit_kinematics_source;
  int32_t orbit_kinematics_source;
  uint8_t has_orbit_kinematics_override;
  pyramid_data_model_agra_OrbitKinematicsType_c orbit_kinematics_override;
} pyramid_data_model_agra_ReferenceAssetKinematicsType_c;

typedef struct pyramid_data_model_agra_EventWindowChoiceType_c {
  uint8_t has_window_angle;
  double window_angle;
  uint8_t has_window_duration;
  pyramid_str_t window_duration;
  uint8_t has_window_radius;
  double window_radius;
} pyramid_data_model_agra_EventWindowChoiceType_c;

typedef struct pyramid_data_model_agra_RepetitionEventBasedType_c {
  pyramid_data_model_agra_RepetitionEventType_c event;
  uint8_t has_event_offset;
  pyramid_data_model_agra_EventOffsetChoiceType_c event_offset;
  uint8_t has_event_window;
  pyramid_data_model_agra_EventWindowChoiceType_c event_window;
  uint8_t has_alternate_reference_object;
  pyramid_data_model_agra_ReferenceAssetKinematicsType_c alternate_reference_object;
  uint32_t repetition_attempts;
} pyramid_data_model_agra_RepetitionEventBasedType_c;

typedef struct pyramid_data_model_agra_RepetitionFiniteType_c {
  uint32_t repetition_attempts;
  pyramid_str_t repetition_interval;
} pyramid_data_model_agra_RepetitionFiniteType_c;

typedef struct pyramid_data_model_agra_RepetitionContinuousType_c {
  pyramid_str_t maximum_interrupt_duration;
} pyramid_data_model_agra_RepetitionContinuousType_c;

typedef struct pyramid_data_model_agra_RepetitionPeriodicType_c {
  pyramid_str_t repetition_interval;
} pyramid_data_model_agra_RepetitionPeriodicType_c;

typedef struct pyramid_data_model_agra_RepetitionTimeBasedType_c {
  uint8_t has_continuous;
  pyramid_data_model_agra_RepetitionContinuousType_c continuous;
  uint8_t has_finite;
  pyramid_data_model_agra_RepetitionFiniteType_c finite;
  uint8_t has_periodic;
  pyramid_data_model_agra_RepetitionPeriodicType_c periodic;
} pyramid_data_model_agra_RepetitionTimeBasedType_c;

typedef struct pyramid_data_model_agra_RepetitionType_c {
  uint8_t has_time_based;
  pyramid_data_model_agra_RepetitionTimeBasedType_c time_based;
  uint8_t has_event_based;
  pyramid_data_model_agra_RepetitionEventBasedType_c event_based;
} pyramid_data_model_agra_RepetitionType_c;

typedef struct pyramid_data_model_agra_RepetitionConstraintsType_c {
  pyramid_data_model_agra_RepetitionType_c repetition;
  uint8_t has_time_window;
  pyramid_data_model_agra_TimeWindowType_c time_window;
} pyramid_data_model_agra_RepetitionConstraintsType_c;

typedef struct pyramid_data_model_agra_TimingConstraintsType_c {
  uint8_t has_as_soon_as_possible;
  pyramid_str_t as_soon_as_possible;
  uint8_t has_time_window;
  pyramid_data_model_agra_TimeWindowType_c time_window;
  uint8_t has_weekday_interval;
  pyramid_data_model_agra_WeekdayIntervalType_c weekday_interval;
  uint8_t has_repetitive;
  pyramid_data_model_agra_RepetitionConstraintsType_c repetitive;
} pyramid_data_model_agra_TimingConstraintsType_c;

typedef struct pyramid_data_model_agra_RequirementTimingType_c {
  int32_t timing_type;
  pyramid_slice_t timing;
} pyramid_data_model_agra_RequirementTimingType_c;

typedef struct pyramid_data_model_agra_MA_ShotDoctrineParametersType_c {
  uint8_t has_use_range_turn_and_run;
  uint8_t use_range_turn_and_run;
  uint8_t has_digital_maneuvering_cue_angle;
  double digital_maneuvering_cue_angle;
  uint8_t has_missiles_per_contact;
  int32_t missiles_per_contact;
  uint8_t has_shoot_nlt;
  double shoot_nlt;
  uint8_t has_shoot_net;
  double shoot_net;
  uint8_t has_enable_target_recalculation;
  uint8_t enable_target_recalculation;
} pyramid_data_model_agra_MA_ShotDoctrineParametersType_c;

typedef struct pyramid_data_model_agra_ComparableRankingType_c {
  uint32_t priority;
  uint32_t precedence_within_priority;
} pyramid_data_model_agra_ComparableRankingType_c;

typedef struct pyramid_data_model_agra_AnglePairType_c {
  double min;
  double max;
} pyramid_data_model_agra_AnglePairType_c;

typedef struct pyramid_data_model_agra_ReferenceFrameID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ReferenceFrameID_Type_c;

typedef struct pyramid_data_model_agra_AltitudeReferenceType_c {
  int32_t altitude_reference;
  double altitude;
} pyramid_data_model_agra_AltitudeReferenceType_c;

typedef struct pyramid_data_model_agra_AltitudeOffsetReferenceType_c {
  int32_t altitude_offset_reference;
  double altitude_offset;
} pyramid_data_model_agra_AltitudeOffsetReferenceType_c;

typedef struct pyramid_data_model_agra_Z_ChoiceType_c {
  uint8_t has_z;
  double z;
  uint8_t has_altitude_offset;
  pyramid_data_model_agra_AltitudeOffsetReferenceType_c altitude_offset;
  uint8_t has_absolute_altitude;
  pyramid_data_model_agra_AltitudeReferenceType_c absolute_altitude;
} pyramid_data_model_agra_Z_ChoiceType_c;

typedef struct pyramid_data_model_agra_RelativeOffset2D_Type_c {
  int32_t rotation;
  int32_t xy_offsets;
  double x;
  double y;
  uint8_t has_z;
  pyramid_data_model_agra_Z_ChoiceType_c z;
} pyramid_data_model_agra_RelativeOffset2D_Type_c;

typedef struct pyramid_data_model_agra_Point2D_RelativeType_c {
  pyramid_data_model_agra_ReferenceFrameID_Type_c reference_frame_id;
  uint8_t has_relative_offset;
  pyramid_data_model_agra_RelativeOffset2D_Type_c relative_offset;
} pyramid_data_model_agra_Point2D_RelativeType_c;

typedef struct pyramid_data_model_agra_AltitudeRangeType_c {
  double min;
  double max;
} pyramid_data_model_agra_AltitudeRangeType_c;

typedef struct pyramid_data_model_agra_Point2D_Type_c {
  double latitude;
  double longitude;
  uint8_t has_altitude;
  double altitude;
  uint8_t has_altitude_range;
  pyramid_data_model_agra_AltitudeRangeType_c altitude_range;
  uint8_t has_altitude_reference;
  int32_t altitude_reference;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_Point2D_Type_c;

typedef struct pyramid_data_model_agra_PointChoiceType_c {
  uint8_t has_absolute_point;
  pyramid_data_model_agra_Point2D_Type_c absolute_point;
  uint8_t has_relative_point;
  pyramid_data_model_agra_Point2D_RelativeType_c relative_point;
} pyramid_data_model_agra_PointChoiceType_c;

typedef struct pyramid_data_model_agra_EllipseType_c {
  double semi_major_axis_length;
  double semi_minor_axis_length;
  double orientation;
} pyramid_data_model_agra_EllipseType_c;

typedef struct pyramid_data_model_agra_LocatedEllipseType_c {
  double semi_major_axis_length;  /* from EllipseType */
  double semi_minor_axis_length;  /* from EllipseType */
  double orientation;  /* from EllipseType */
  pyramid_data_model_agra_PointChoiceType_c center_point_choice;
} pyramid_data_model_agra_LocatedEllipseType_c;

typedef struct pyramid_data_model_agra_RectangleType_c {
  double width;
  double height;
  double orientation;
} pyramid_data_model_agra_RectangleType_c;

typedef struct pyramid_data_model_agra_LocatedRectangleType_c {
  double width;  /* from RectangleType */
  double height;  /* from RectangleType */
  double orientation;  /* from RectangleType */
  pyramid_data_model_agra_PointChoiceType_c center_point_choice;
} pyramid_data_model_agra_LocatedRectangleType_c;

typedef struct pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c;

typedef struct pyramid_data_model_agra_PolygonRelativeType_c {
  pyramid_data_model_agra_ReferenceFrameID_Type_c reference_frame_id;
  pyramid_slice_t vertex;
} pyramid_data_model_agra_PolygonRelativeType_c;

typedef struct pyramid_data_model_agra_PolygonPointChoiceType_c {
  uint8_t has_point2_d;
  pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c point2_d;
  uint8_t has_relative_polygon;
  pyramid_data_model_agra_PolygonRelativeType_c relative_polygon;
} pyramid_data_model_agra_PolygonPointChoiceType_c;

typedef struct pyramid_data_model_agra_PolygonType_c {
  int32_t line_projection;
  pyramid_data_model_agra_PolygonPointChoiceType_c bounding_polygon;
  pyramid_slice_t interior_polygon;
} pyramid_data_model_agra_PolygonType_c;

typedef struct pyramid_data_model_agra_SlantRangeConstraintsType_c {
  double min;
  double max;
  uint8_t has_absolute;
  uint8_t absolute;
} pyramid_data_model_agra_SlantRangeConstraintsType_c;

typedef struct pyramid_data_model_agra_SlantRangeAreaType_c {
  pyramid_data_model_agra_SlantRangeConstraintsType_c radius;
  pyramid_data_model_agra_AnglePairType_c azimuth_extent;
  double orientation;
  pyramid_data_model_agra_PointChoiceType_c point_choice;
} pyramid_data_model_agra_SlantRangeAreaType_c;

typedef struct pyramid_data_model_agra_AreaChoiceType_c {
  uint8_t has_polygon;
  pyramid_data_model_agra_PolygonType_c polygon;
  uint8_t has_ellipse;
  pyramid_data_model_agra_LocatedEllipseType_c ellipse;
  uint8_t has_rectangle;
  pyramid_data_model_agra_LocatedRectangleType_c rectangle;
  uint8_t has_slant_range_area;
  pyramid_data_model_agra_SlantRangeAreaType_c slant_range_area;
} pyramid_data_model_agra_AreaChoiceType_c;

typedef struct pyramid_data_model_agra_ZoneType_c {
  pyramid_data_model_agra_AreaChoiceType_c shape;
  uint8_t has_min_altitude;
  double min_altitude;
  uint8_t has_max_altitude;
  double max_altitude;
  uint8_t has_altitude_reference;
  int32_t altitude_reference;
} pyramid_data_model_agra_ZoneType_c;

typedef struct pyramid_data_model_agra_QueryPET_c {
} pyramid_data_model_agra_QueryPET_c;

typedef struct pyramid_data_model_agra_GeoFiltersQueryType_c {
  pyramid_data_model_agra_ZoneType_c geo_filter_zone;
  int32_t relation_to_geo_filter_zone;
} pyramid_data_model_agra_GeoFiltersQueryType_c;

typedef struct pyramid_data_model_agra_IncRaPeriodVolumeType_c {
  double delta_inclination;
  double delta_right_ascension;
  pyramid_str_t delta_period;
} pyramid_data_model_agra_IncRaPeriodVolumeType_c;

typedef struct pyramid_data_model_agra_SphereType_c {
  double radius;
} pyramid_data_model_agra_SphereType_c;

typedef struct pyramid_data_model_agra_DomeType_c {
  double radius;
} pyramid_data_model_agra_DomeType_c;

typedef struct pyramid_data_model_agra_CylinderType_c {
  uint8_t has_length;
  double length;
  double radius;
} pyramid_data_model_agra_CylinderType_c;

typedef struct pyramid_data_model_agra_EllipsoidType_c {
  double semi_major_axis_length_a;
  double semi_minor_axis_length_b;
  double semi_minor_axis_length_c;
  pyramid_data_model_agra_QuaternionType_c attitude;
} pyramid_data_model_agra_EllipsoidType_c;

typedef struct pyramid_data_model_agra_RectangularConeType_c {
  double length;
  double width;
  uint8_t has_range;
  double range;
} pyramid_data_model_agra_RectangularConeType_c;

typedef struct pyramid_data_model_agra_ConeType_c {
  double half_angle;
  uint8_t has_range;
  double range;
} pyramid_data_model_agra_ConeType_c;

typedef struct pyramid_data_model_agra_AlongOrbitalArcDeltaType_c {
  uint8_t has_mean_anomaly;
  double mean_anomaly;
  pyramid_str_t time_delta;
} pyramid_data_model_agra_AlongOrbitalArcDeltaType_c;

typedef struct pyramid_data_model_agra_ArcVolumeType_c {
  double radial_delta;
  pyramid_data_model_agra_AlongOrbitalArcDeltaType_c along_orbital_arc_delta;
  double cross_track_delta;
} pyramid_data_model_agra_ArcVolumeType_c;

typedef struct pyramid_data_model_agra_Shape3D_ChoiceType_c {
  uint8_t has_sphere;
  pyramid_data_model_agra_SphereType_c sphere;
  uint8_t has_dome;
  pyramid_data_model_agra_DomeType_c dome;
  uint8_t has_ellipsoid;
  pyramid_data_model_agra_EllipsoidType_c ellipsoid;
  uint8_t has_cylinder;
  pyramid_data_model_agra_CylinderType_c cylinder;
  uint8_t has_cone;
  pyramid_data_model_agra_ConeType_c cone;
  uint8_t has_rectangular_cone;
  pyramid_data_model_agra_RectangularConeType_c rectangular_cone;
  uint8_t has_arc_volume;
  pyramid_data_model_agra_ArcVolumeType_c arc_volume;
  uint8_t has_inc_ra_period_volume;
  pyramid_data_model_agra_IncRaPeriodVolumeType_c inc_ra_period_volume;
} pyramid_data_model_agra_Shape3D_ChoiceType_c;

typedef struct pyramid_data_model_agra_Velocity2D_Type_c {
  double north_speed;
  double east_speed;
  uint8_t has_down_speed;
  double down_speed;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_Velocity2D_Type_c;

typedef struct pyramid_data_model_agra_OpVolumeKinematicsType_c {
  pyramid_data_model_agra_PointChoiceType_c point_choice;
  uint8_t has_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c velocity;
} pyramid_data_model_agra_OpVolumeKinematicsType_c;

typedef struct pyramid_data_model_agra_AngleHalfPairType_c {
  double min;
  double max;
} pyramid_data_model_agra_AngleHalfPairType_c;

typedef struct pyramid_data_model_agra_UncertaintyType_c {
  pyramid_data_model_agra_EllipseType_c uncertainty_ellipse;
  uint8_t has_uncertainty_vertical;
  double uncertainty_vertical;
  uint8_t has_uncertainty_tilt;
  double uncertainty_tilt;
} pyramid_data_model_agra_UncertaintyType_c;

typedef struct pyramid_data_model_agra_Velocity2D_UncertaintyType_c {
  double north_speed;  /* from Velocity2D_Type */
  double east_speed;  /* from Velocity2D_Type */
  uint8_t has_down_speed;
  double down_speed;  /* from Velocity2D_Type */
  pyramid_str_t timestamp;  /* from Velocity2D_Type */
  uint8_t has_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c uncertainty;
} pyramid_data_model_agra_Velocity2D_UncertaintyType_c;

typedef struct pyramid_data_model_agra_Acceleration3D_Type_c {
  double north_acceleration;
  double east_acceleration;
  double down_acceleration;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_Acceleration3D_Type_c;

typedef struct pyramid_data_model_agra_OrientationType_c {
  double yaw;
  double pitch;
  double roll;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_OrientationType_c;

typedef struct pyramid_data_model_agra_OpPointID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OpPointID_Type_c;

typedef struct pyramid_data_model_agra_ReferenceObjectType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_op_point_id;
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
} pyramid_data_model_agra_ReferenceObjectType_c;

typedef struct pyramid_data_model_agra_Point2D_ReferenceType_c {
  pyramid_data_model_agra_PointChoiceType_c point_choice;
  uint8_t has_reference_object;
  pyramid_data_model_agra_ReferenceObjectType_c reference_object;
} pyramid_data_model_agra_Point2D_ReferenceType_c;

typedef struct pyramid_data_model_agra_UnitVectorType_c {
  float x;
  float y;
  float z;
} pyramid_data_model_agra_UnitVectorType_c;

typedef struct pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c {
  double los_cross_vertical_angle_rate_error;
  double los_cross_horizontal_angle_rate_error;
} pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c;

typedef struct pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c {
  double los_cross_vertical_angle_error;
  double los_cross_horizontal_angle_error;
} pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c;

typedef struct pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c {
  int32_t los_reference_frame;
  uint8_t has_capability_id;
  pyramid_data_model_agra_CapabilityID_Type_c capability_id;
  pyramid_data_model_agra_UnitVectorType_c los_unit_vector;
  uint8_t has_los_reference_frame_roll;
  double los_reference_frame_roll;
  uint8_t has_relative_angle_uncertainty_los3_d;
  pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c relative_angle_uncertainty_los3_d;
  uint8_t has_los_reference_frame_roll_rate;
  double los_reference_frame_roll_rate;
  uint8_t has_los_cross_vertical_angle_rate;
  double los_cross_vertical_angle_rate;
  uint8_t has_los_cross_horizontal_angle_rate;
  double los_cross_horizontal_angle_rate;
  uint8_t has_relative_angle_rate_uncertainty_los3_d;
  pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c relative_angle_rate_uncertainty_los3_d;
} pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c;

typedef struct pyramid_data_model_agra_LOS3D_CovarianceType_c {
  double rv_rv;
  double rv_rh;
  double rh_rh;
  uint8_t has_ps_rv;
  double ps_rv;
  uint8_t has_ps_rh;
  double ps_rh;
  uint8_t has_ps_ps;
  double ps_ps;
} pyramid_data_model_agra_LOS3D_CovarianceType_c;

typedef struct pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c {
  double slant_range;
  uint8_t has_slant_range_error;
  double slant_range_error;
  uint8_t has_slant_range_rate;
  double slant_range_rate;
  uint8_t has_slant_range_rate_error;
  double slant_range_rate_error;
} pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c;

typedef struct pyramid_data_model_agra_LOS3D_KinematicsType_c {
  pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c relative_angles_los3_d;
  uint8_t has_relative_slant_range_los3_d;
  pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c relative_slant_range_los3_d;
  uint8_t has_los3_d_orientation_covariance;
  pyramid_data_model_agra_LOS3D_CovarianceType_c los3_d_orientation_covariance;
} pyramid_data_model_agra_LOS3D_KinematicsType_c;

typedef struct pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c {
  uint8_t has_coscone_y_slant_range_rate;
  double coscone_y_slant_range_rate;
  uint8_t has_coscone_z_slant_range_rate;
  double coscone_z_slant_range_rate;
  uint8_t has_slant_range_coscone_y_rate;
  double slant_range_coscone_y_rate;
  uint8_t has_slant_range_coscone_z_rate;
  double slant_range_coscone_z_rate;
} pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c;

typedef struct pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c {
  uint8_t has_slant_range_coscone_y;
  double slant_range_coscone_y;
  uint8_t has_slant_range_coscone_z;
  double slant_range_coscone_z;
  uint8_t has_rates;
  pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c rates;
} pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c;

typedef struct pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c {
  pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c covariance;
} pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c;

typedef struct pyramid_data_model_agra_ArrivalDataVarianceType_c {
  uint8_t has_time_of_arrival;
  double time_of_arrival;
  uint8_t has_time_difference_of_arrival;
  double time_difference_of_arrival;
  uint8_t has_frequency_of_arrival;
  double frequency_of_arrival;
  uint8_t has_frequency_difference_of_arrival;
  double frequency_difference_of_arrival;
} pyramid_data_model_agra_ArrivalDataVarianceType_c;

typedef struct pyramid_data_model_agra_ArrivalDataUncertaintyType_c {
  pyramid_data_model_agra_ArrivalDataVarianceType_c variance;
} pyramid_data_model_agra_ArrivalDataUncertaintyType_c;

typedef struct pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c {
  uint8_t has_slant_range_azimuth_rate;
  double slant_range_azimuth_rate;
  uint8_t has_slant_range_elevation_rate;
  double slant_range_elevation_rate;
  uint8_t has_azimuth_slant_range_rate;
  double azimuth_slant_range_rate;
  uint8_t has_elevation_slant_range_rate;
  double elevation_slant_range_rate;
} pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c;

typedef struct pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c {
  uint8_t has_slant_range_azimuth;
  double slant_range_azimuth;
  uint8_t has_slant_range_elevation;
  double slant_range_elevation;
  uint8_t has_rates;
  pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c rates;
} pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c;

typedef struct pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c {
  pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c covariance;
} pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c;

typedef struct pyramid_data_model_agra_LOS_CovariancesRatesType_c {
  uint8_t has_azimuth_azimuth_rate;
  double azimuth_azimuth_rate;
  uint8_t has_elevation_elevation_rate;
  double elevation_elevation_rate;
  uint8_t has_azimuth_elevation_rate;
  double azimuth_elevation_rate;
  uint8_t has_elevation_azimuth_rate;
  double elevation_azimuth_rate;
  uint8_t has_azimuth_rate_elevation_rate;
  double azimuth_rate_elevation_rate;
} pyramid_data_model_agra_LOS_CovariancesRatesType_c;

typedef struct pyramid_data_model_agra_LOS_CovarianceType_c {
  uint8_t has_azimuth_elevation;
  double azimuth_elevation;
  uint8_t has_rates;
  pyramid_data_model_agra_LOS_CovariancesRatesType_c rates;
} pyramid_data_model_agra_LOS_CovarianceType_c;

typedef struct pyramid_data_model_agra_LOS_VarianceRatesType_c {
  uint8_t has_azimuth_rate;
  double azimuth_rate;
  uint8_t has_elevation_rate;
  double elevation_rate;
} pyramid_data_model_agra_LOS_VarianceRatesType_c;

typedef struct pyramid_data_model_agra_LOS_VarianceType_c {
  uint8_t has_azimuth;
  double azimuth;
  uint8_t has_elevation;
  double elevation;
  uint8_t has_rates;
  pyramid_data_model_agra_LOS_VarianceRatesType_c rates;
} pyramid_data_model_agra_LOS_VarianceType_c;

typedef struct pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c {
  pyramid_data_model_agra_LOS_VarianceType_c variance;
  uint8_t has_covariance;
  pyramid_data_model_agra_LOS_CovarianceType_c covariance;
} pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c;

typedef struct pyramid_data_model_agra_ConeAngleCovarianceRatesType_c {
  uint8_t has_coscone_y_coscone_y_rate;
  double coscone_y_coscone_y_rate;
  uint8_t has_coscone_y_coscone_z_rate;
  double coscone_y_coscone_z_rate;
  uint8_t has_coscone_z_coscone_y_rate;
  double coscone_z_coscone_y_rate;
  uint8_t has_coscone_z_coscone_z_rate;
  double coscone_z_coscone_z_rate;
  uint8_t has_coscone_y_rate_coscone_z_rate;
  double coscone_y_rate_coscone_z_rate;
} pyramid_data_model_agra_ConeAngleCovarianceRatesType_c;

typedef struct pyramid_data_model_agra_ConeAngleCovarianceType_c {
  uint8_t has_coscone_y_coscone_z;
  double coscone_y_coscone_z;
  uint8_t has_rates;
  pyramid_data_model_agra_ConeAngleCovarianceRatesType_c rates;
} pyramid_data_model_agra_ConeAngleCovarianceType_c;

typedef struct pyramid_data_model_agra_ConeAngleVarianceRatesType_c {
  uint8_t has_coscone_y_rate;
  double coscone_y_rate;
  uint8_t has_coscone_z_rate;
  double coscone_z_rate;
} pyramid_data_model_agra_ConeAngleVarianceRatesType_c;

typedef struct pyramid_data_model_agra_ConeAngleVarianceType_c {
  uint8_t has_coscone_y;
  double coscone_y;
  uint8_t has_coscone_z;
  double coscone_z;
  uint8_t has_rates;
  pyramid_data_model_agra_ConeAngleVarianceRatesType_c rates;
} pyramid_data_model_agra_ConeAngleVarianceType_c;

typedef struct pyramid_data_model_agra_ConeAngleUncertaintyType_c {
  pyramid_data_model_agra_ConeAngleVarianceType_c variance;
  uint8_t has_covariance;
  pyramid_data_model_agra_ConeAngleCovarianceType_c covariance;
} pyramid_data_model_agra_ConeAngleUncertaintyType_c;

typedef struct pyramid_data_model_agra_SlantRangeCovarianceType_c {
  double slant_range_slant_range_rate;
} pyramid_data_model_agra_SlantRangeCovarianceType_c;

typedef struct pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c {
  uint8_t has_slant_range_rate;
  double slant_range_rate;
  uint8_t has_slant_range_acceleration;
  double slant_range_acceleration;
  uint8_t has_delta_slant_range;
  double delta_slant_range;
  uint8_t has_delta_slant_range_rate;
  double delta_slant_range_rate;
} pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c;

typedef struct pyramid_data_model_agra_SlantRangeVarianceType_c {
  uint8_t has_slant_range;
  double slant_range;
  uint8_t has_rates_and_acceleration;
  pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c rates_and_acceleration;
} pyramid_data_model_agra_SlantRangeVarianceType_c;

typedef struct pyramid_data_model_agra_SlantRangeUncertaintyType_c {
  pyramid_data_model_agra_SlantRangeVarianceType_c variance;
  uint8_t has_covariance;
  pyramid_data_model_agra_SlantRangeCovarianceType_c covariance;
} pyramid_data_model_agra_SlantRangeUncertaintyType_c;

typedef struct pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c {
  uint8_t has_los_uncertainty;
  pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c los_uncertainty;
  uint8_t has_cone_angle_uncertainty;
  pyramid_data_model_agra_ConeAngleUncertaintyType_c cone_angle_uncertainty;
  uint8_t has_arrival_data_uncertainty;
  pyramid_data_model_agra_ArrivalDataUncertaintyType_c arrival_data_uncertainty;
  uint8_t has_slant_range_uncertainty;
  pyramid_data_model_agra_SlantRangeUncertaintyType_c slant_range_uncertainty;
  uint8_t has_cone_angle_slant_range_uncertainty;
  pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c cone_angle_slant_range_uncertainty;
  uint8_t has_los_slant_range_uncertainty;
  pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c los_slant_range_uncertainty;
} pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c;

typedef struct pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c {
  uint8_t has_root_mean_square;
  pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c root_mean_square;
  uint8_t has_bias;
  pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c bias;
  uint8_t has_random;
  pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c random;
} pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c;

typedef struct pyramid_data_model_agra_ArrivalDataType_c {
  pyramid_str_t time_of_arrival;
  pyramid_str_t time_difference_of_arrival;
  uint8_t has_frequency_of_arrival;
  double frequency_of_arrival;
  uint8_t has_frequency_difference_of_arrival;
  double frequency_difference_of_arrival;
} pyramid_data_model_agra_ArrivalDataType_c;

typedef struct pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c {
  uint8_t has_slant_range_rate;
  double slant_range_rate;
  uint8_t has_delta_slant_range_rate;
  double delta_slant_range_rate;
  uint8_t has_slant_range_acceleration;
  double slant_range_acceleration;
} pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c;

typedef struct pyramid_data_model_agra_SlantRangeType_c {
  uint8_t has_slant_range;
  double slant_range;
  uint8_t has_delta_slant_range;
  double delta_slant_range;
  uint8_t has_rates_and_acceleration;
  pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c rates_and_acceleration;
} pyramid_data_model_agra_SlantRangeType_c;

typedef struct pyramid_data_model_agra_LOS_AzElRatesType_c {
  uint8_t has_azimuth_rate;
  double azimuth_rate;
  uint8_t has_elevation_rate;
  double elevation_rate;
} pyramid_data_model_agra_LOS_AzElRatesType_c;

typedef struct pyramid_data_model_agra_LOS_AzElType_c {
  uint8_t has_azimuth;
  double azimuth;
  uint8_t has_elevation;
  double elevation;
  uint8_t has_rates;
  pyramid_data_model_agra_LOS_AzElRatesType_c rates;
} pyramid_data_model_agra_LOS_AzElType_c;

typedef struct pyramid_data_model_agra_ConeAngleRatesType_c {
  uint8_t has_coscone_y_rate;
  double coscone_y_rate;
  uint8_t has_coscone_z_rate;
  double coscone_z_rate;
} pyramid_data_model_agra_ConeAngleRatesType_c;

typedef struct pyramid_data_model_agra_ConeAngleType_c {
  uint8_t has_coscone_y;
  double coscone_y;
  uint8_t has_coscone_z;
  double coscone_z;
  uint8_t has_rates;
  pyramid_data_model_agra_ConeAngleRatesType_c rates;
} pyramid_data_model_agra_ConeAngleType_c;

typedef struct pyramid_data_model_agra_LOS_MeasurementType_c {
  uint8_t has_line_of_sight;
  pyramid_data_model_agra_LOS_AzElType_c line_of_sight;
  uint8_t has_cone_angle;
  pyramid_data_model_agra_ConeAngleType_c cone_angle;
  uint8_t has_arrival_data;
  pyramid_data_model_agra_ArrivalDataType_c arrival_data;
  uint8_t has_slant_range;
  pyramid_data_model_agra_SlantRangeType_c slant_range;
} pyramid_data_model_agra_LOS_MeasurementType_c;

typedef struct pyramid_data_model_agra_LOS_UncertaintyType_c {
  uint8_t has_slant_range_error;
  double slant_range_error;
  uint8_t has_azimuth_error;
  double azimuth_error;
  uint8_t has_elevation_error;
  double elevation_error;
} pyramid_data_model_agra_LOS_UncertaintyType_c;

typedef struct pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c {
  pyramid_data_model_agra_LOS_MeasurementType_c measurement;
  uint8_t has_covariance;
  pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c covariance;
  uint8_t has_los_uncertainty;
  pyramid_data_model_agra_LOS_UncertaintyType_c los_uncertainty;
} pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c;

typedef struct pyramid_data_model_agra_LineOfSightChoiceType_c {
  uint8_t has_los_az_el;
  pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c los_az_el;
  uint8_t has_los3_d_kinematics;
  pyramid_data_model_agra_LOS3D_KinematicsType_c los3_d_kinematics;
} pyramid_data_model_agra_LineOfSightChoiceType_c;

typedef struct pyramid_data_model_agra_RelativePositionType_c {
  pyramid_data_model_agra_Point2D_ReferenceType_c reference_point;
  uint8_t has_los;
  pyramid_data_model_agra_LineOfSightChoiceType_c los;
} pyramid_data_model_agra_RelativePositionType_c;

typedef struct pyramid_data_model_agra_Point2D_ReportedType_c {
  double latitude;  /* from Point2D_Type */
  double longitude;  /* from Point2D_Type */
  uint8_t has_altitude;
  double altitude;  /* from Point2D_Type */
  uint8_t has_altitude_range;
  pyramid_data_model_agra_AltitudeRangeType_c altitude_range;  /* from Point2D_Type */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from Point2D_Type */
  pyramid_str_t timestamp;  /* from Point2D_Type */
  uint8_t has_hae_adjustment;
  double hae_adjustment;
  uint8_t has_altitude_source;
  int32_t altitude_source;
  uint8_t has_depth_contact;
  int32_t depth_contact;
} pyramid_data_model_agra_Point2D_ReportedType_c;

typedef struct pyramid_data_model_agra_FixedPositionType_c {
  pyramid_data_model_agra_Point2D_ReportedType_c fixed_point;
  uint8_t has_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c uncertainty;
  uint8_t has_datum_error;
  double datum_error;
} pyramid_data_model_agra_FixedPositionType_c;

typedef struct pyramid_data_model_agra_EntityPositionType_c {
  uint8_t has_fixed_position_type;
  pyramid_data_model_agra_FixedPositionType_c fixed_position_type;
  uint8_t has_relative_position;
  pyramid_data_model_agra_RelativePositionType_c relative_position;
  uint8_t has_zone;
  pyramid_data_model_agra_ZoneType_c zone;
} pyramid_data_model_agra_EntityPositionType_c;

typedef struct pyramid_data_model_agra_VelocityAccelerationCovarianceType_c {
  double vn_an;
  double vn_ae;
  uint8_t has_vn_ad;
  double vn_ad;
  double ve_an;
  double ve_ae;
  uint8_t has_ve_ad;
  double ve_ad;
  uint8_t has_vd_an;
  double vd_an;
  uint8_t has_vd_ae;
  double vd_ae;
  uint8_t has_vd_ad;
  double vd_ad;
} pyramid_data_model_agra_VelocityAccelerationCovarianceType_c;

typedef struct pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c {
  double an_an;
  double an_ae;
  uint8_t has_an_ad;
  double an_ad;
  double ae_ae;
  uint8_t has_ae_ad;
  double ae_ad;
  uint8_t has_ad_ad;
  double ad_ad;
} pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c;

typedef struct pyramid_data_model_agra_PositionAccelerationCovarianceType_c {
  double pn_an;
  double pn_ae;
  uint8_t has_pn_ad;
  double pn_ad;
  double pe_an;
  double pe_ae;
  uint8_t has_pe_ad;
  double pe_ad;
  uint8_t has_pd_an;
  double pd_an;
  uint8_t has_pd_ae;
  double pd_ae;
  uint8_t has_pd_ad;
  double pd_ad;
} pyramid_data_model_agra_PositionAccelerationCovarianceType_c;

typedef struct pyramid_data_model_agra_PositionVelocityCovarianceType_c {
  double pn_vn;
  double pn_ve;
  uint8_t has_pn_vd;
  double pn_vd;
  double pe_vn;
  double pe_ve;
  uint8_t has_pe_vd;
  double pe_vd;
  uint8_t has_pd_vn;
  double pd_vn;
  uint8_t has_pd_ve;
  double pd_ve;
  uint8_t has_pd_vd;
  double pd_vd;
} pyramid_data_model_agra_PositionVelocityCovarianceType_c;

typedef struct pyramid_data_model_agra_VelocityVelocityCovarianceType_c {
  double vn_vn;
  double vn_ve;
  uint8_t has_vn_vd;
  double vn_vd;
  double ve_ve;
  uint8_t has_ve_vd;
  double ve_vd;
  uint8_t has_vd_vd;
  double vd_vd;
} pyramid_data_model_agra_VelocityVelocityCovarianceType_c;

typedef struct pyramid_data_model_agra_PositionPositionCovarianceType_c {
  double pn_pn;
  double pn_pe;
  uint8_t has_pn_pd;
  double pn_pd;
  double pe_pe;
  uint8_t has_pe_pd;
  double pe_pd;
  uint8_t has_pd_pd;
  double pd_pd;
} pyramid_data_model_agra_PositionPositionCovarianceType_c;

typedef struct pyramid_data_model_agra_PositionAndVelocityCovarianceType_c {
  pyramid_data_model_agra_PositionPositionCovarianceType_c position_position;
  uint8_t has_position_velocity;
  pyramid_data_model_agra_PositionVelocityCovarianceType_c position_velocity;
  uint8_t has_velocity_velocity;
  pyramid_data_model_agra_VelocityVelocityCovarianceType_c velocity_velocity;
} pyramid_data_model_agra_PositionAndVelocityCovarianceType_c;

typedef struct pyramid_data_model_agra_StateCovarianceNED_Type_c {
  pyramid_data_model_agra_PositionPositionCovarianceType_c position_position;  /* from PositionAndVelocityCovarianceType */
  uint8_t has_position_velocity;
  pyramid_data_model_agra_PositionVelocityCovarianceType_c position_velocity;  /* from PositionAndVelocityCovarianceType */
  uint8_t has_velocity_velocity;
  pyramid_data_model_agra_VelocityVelocityCovarianceType_c velocity_velocity;  /* from PositionAndVelocityCovarianceType */
  uint8_t has_acceleration_acceleration;
  pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c acceleration_acceleration;
  uint8_t has_position_acceleration;
  pyramid_data_model_agra_PositionAccelerationCovarianceType_c position_acceleration;
  uint8_t has_velocity_acceleration;
  pyramid_data_model_agra_VelocityAccelerationCovarianceType_c velocity_acceleration;
} pyramid_data_model_agra_StateCovarianceNED_Type_c;

typedef struct pyramid_data_model_agra_OrientationCovarianceType_c {
  double rr_rr;
  double rr_rp;
  double rr_ry;
  double rp_rp;
  double rp_ry;
  double ry_ry;
} pyramid_data_model_agra_OrientationCovarianceType_c;

typedef struct pyramid_data_model_agra_KinematicsType_c {
  pyramid_str_t kinematics_time_stamp;
  pyramid_data_model_agra_EntityPositionType_c position;
  uint8_t has_velocity;
  pyramid_data_model_agra_Velocity2D_UncertaintyType_c velocity;
  uint8_t has_acceleration;
  pyramid_data_model_agra_Acceleration3D_Type_c acceleration;
  uint8_t has_state_covariance;
  pyramid_data_model_agra_StateCovarianceNED_Type_c state_covariance;
  uint8_t has_orientation;
  pyramid_data_model_agra_OrientationType_c orientation;
  uint8_t has_orientation_covariance;
  pyramid_data_model_agra_OrientationCovarianceType_c orientation_covariance;
} pyramid_data_model_agra_KinematicsType_c;

typedef struct pyramid_data_model_agra_KinematicsMultiStandardType_c {
  uint8_t has_orbital;
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c orbital;
  uint8_t has_wgs;
  pyramid_data_model_agra_KinematicsType_c wgs;
  uint8_t has_delta_orbital_plane_tolerance;
  pyramid_data_model_agra_AngleHalfPairType_c delta_orbital_plane_tolerance;
} pyramid_data_model_agra_KinematicsMultiStandardType_c;

typedef struct pyramid_data_model_agra_KinematicsOptionsType_c {
  uint8_t has_reference_asset;
  pyramid_data_model_agra_AssetType_c reference_asset;
  uint8_t has_kinematics_override;
  pyramid_data_model_agra_KinematicsMultiStandardType_c kinematics_override;
} pyramid_data_model_agra_KinematicsOptionsType_c;

typedef struct pyramid_data_model_agra_RTN_LocalPositionType_c {
  pyramid_data_model_agra_RTN_PositionType_c position;
  pyramid_data_model_agra_QuaternionType_c attitude;
  pyramid_data_model_agra_KinematicsOptionsType_c relative_kinematics;
} pyramid_data_model_agra_RTN_LocalPositionType_c;

typedef struct pyramid_data_model_agra_KinematicsChoiceType_c {
  uint8_t has_volume_kinematics;
  pyramid_data_model_agra_OpVolumeKinematicsType_c volume_kinematics;
  uint8_t has_orbital_kinematics;
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c orbital_kinematics;
  uint8_t has_local_body_position;
  pyramid_data_model_agra_RTN_LocalPositionType_c local_body_position;
} pyramid_data_model_agra_KinematicsChoiceType_c;

typedef struct pyramid_data_model_agra_GeometricVolumeType_c {
  pyramid_data_model_agra_Shape3D_ChoiceType_c shape;
  pyramid_data_model_agra_KinematicsChoiceType_c kinematics_choice;
} pyramid_data_model_agra_GeometricVolumeType_c;

typedef struct pyramid_data_model_agra_GeocentricVolumeType_c {
  pyramid_data_model_agra_AltitudeRangeType_c altitude_range;
  pyramid_data_model_agra_AnglePairType_c longitude_range;
  pyramid_data_model_agra_AngleHalfPairType_c latitude_range;
} pyramid_data_model_agra_GeocentricVolumeType_c;

typedef struct pyramid_data_model_agra_OpVolumeType_c {
  uint8_t has_geometric_volume;
  pyramid_data_model_agra_GeometricVolumeType_c geometric_volume;
  uint8_t has_geocentric_volume;
  pyramid_data_model_agra_GeocentricVolumeType_c geocentric_volume;
  uint8_t has_orbit_regime;
  pyramid_data_model_agra_OrbitRegimeType_c orbit_regime;
  uint8_t has_orbit_altitude;
  int32_t orbit_altitude;
  uint8_t has_qualitative;
  int32_t qualitative;
} pyramid_data_model_agra_OpVolumeType_c;

typedef struct pyramid_data_model_agra_OrbitalFiltersQueryType_c {
  pyramid_data_model_agra_OpVolumeType_c orbital_filter_zone;
  int32_t relation_to_orbital_filter_zone;
} pyramid_data_model_agra_OrbitalFiltersQueryType_c;

typedef struct pyramid_data_model_agra_MA_RequirementKinematicConstraintsType_c {
  int32_t applicable_object;
  pyramid_slice_t geo;
  pyramid_slice_t orbital;
  uint8_t has_look_angle_constraints;
  pyramid_data_model_agra_AnglePairType_c look_angle_constraints;
} pyramid_data_model_agra_MA_RequirementKinematicConstraintsType_c;

typedef struct pyramid_data_model_agra_DistanceConstraintsType_c {
  uint8_t has_min;
  double min;
  uint8_t has_max;
  double max;
} pyramid_data_model_agra_DistanceConstraintsType_c;

typedef struct pyramid_data_model_agra_DurationRangeType_c {
  pyramid_str_t min;
  pyramid_str_t max;
} pyramid_data_model_agra_DurationRangeType_c;

typedef struct pyramid_data_model_agra_AccessEventFilterType_c {
  uint8_t has_effort;
  int32_t effort;
  uint8_t has_time_range;
  pyramid_data_model_agra_DateTimeRangeType_c time_range;
  uint8_t has_duration_range;
  pyramid_data_model_agra_DurationRangeType_c duration_range;
  uint8_t has_object_separation_range;
  pyramid_data_model_agra_DistanceConstraintsType_c object_separation_range;
  uint8_t has_unambiguous_time_range;
  pyramid_data_model_agra_DateTimeRangeType_c unambiguous_time_range;
} pyramid_data_model_agra_AccessEventFilterType_c;

typedef struct pyramid_data_model_agra_MA_DesignationFilterType_c {
  pyramid_slice_t designation_type;
  uint8_t has_designation_objective;
  pyramid_data_model_agra_MA_RequirementTaxonomyType_c designation_objective;
} pyramid_data_model_agra_MA_DesignationFilterType_c;

typedef struct pyramid_data_model_agra_MA_PrioritizationType_c {
  pyramid_data_model_agra_ComparableRankingType_c priority_value;
  uint8_t has_requirement_type;
  pyramid_data_model_agra_MA_RequirementTaxonomyType_c requirement_type;
} pyramid_data_model_agra_MA_PrioritizationType_c;

typedef struct pyramid_data_model_agra_MA_PrioritizationListValueType_c {
  int32_t list_type;
  pyramid_data_model_agra_MA_PrioritizationType_c prioritization;
} pyramid_data_model_agra_MA_PrioritizationListValueType_c;

typedef struct pyramid_data_model_agra_EntityIdentityChoiceType_c {
  uint8_t has_standard;
  pyramid_data_model_agra_StandardIdentityType_c standard;
  uint8_t has_environment;
  pyramid_data_model_agra_EnvironmentIdentityType_c environment;
  uint8_t has_platform;
  pyramid_data_model_agra_PlatformIdentityType_c platform;
  uint8_t has_specific;
  pyramid_data_model_agra_SpecificIdentityType_c specific;
  uint8_t has_emitter;
  pyramid_data_model_agra_EmitterIdentityType_c emitter;
  uint8_t has_specific_emitter;
  pyramid_data_model_agra_SpecificEmitterIdentityType_c specific_emitter;
  uint8_t has_specific_vehicle;
  pyramid_data_model_agra_VehicleIdentificationType_c specific_vehicle;
  uint8_t has_specific_facility;
  pyramid_data_model_agra_FacilityIdentificationType_c specific_facility;
  uint8_t has_eob;
  pyramid_data_model_agra_EOB_IdentityType_c eob;
  uint8_t has_weapon;
  pyramid_data_model_agra_StoreType_c weapon;
} pyramid_data_model_agra_EntityIdentityChoiceType_c;

typedef struct pyramid_data_model_agra_IdentityComparisonType_c {
  pyramid_data_model_agra_EntityIdentityChoiceType_c type;
  double confidence;
} pyramid_data_model_agra_IdentityComparisonType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_OrbitalSurveillanceSpecificDataType_c;

typedef struct pyramid_data_model_agra_SMTI_SpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_SMTI_SpecificDataType_c;

typedef struct pyramid_data_model_agra_ESM_SpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_ESM_SpecificDataType_c;

typedef struct pyramid_data_model_agra_SAR_SpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_SAR_SpecificDataType_c;

typedef struct pyramid_data_model_agra_AMTI_SpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_AMTI_SpecificDataType_c;

typedef struct pyramid_data_model_agra_COMINT_SpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_COMINT_SpecificDataType_c;

typedef struct pyramid_data_model_agra_CargoDeliverySpecificDataType_c {
  uint8_t has_capability_type;
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
} pyramid_data_model_agra_CargoDeliverySpecificDataType_c;

typedef struct pyramid_data_model_agra_CapabilityTaxonomyType_c {
  pyramid_slice_t action;
  pyramid_slice_t air_sample;
  pyramid_slice_t amti;
  pyramid_slice_t ao;
  pyramid_slice_t cargo_delivery;
  pyramid_slice_t comint;
  pyramid_slice_t comm_relay;
  uint8_t has_counter_space;
  int32_t counter_space;
  pyramid_slice_t ea;
  pyramid_slice_t effect;
  pyramid_slice_t esm;
  pyramid_slice_t flight;
  pyramid_slice_t orbit_change;
  pyramid_slice_t orbital_surveillance;
  pyramid_slice_t orbital_surveillance_sensor;
  pyramid_slice_t po;
  pyramid_slice_t refuel;
  pyramid_slice_t response;
  pyramid_slice_t sar;
  pyramid_slice_t smti;
  pyramid_slice_t strike;
  pyramid_slice_t system_deployment;
  pyramid_slice_t tactical_order;
  uint8_t has_weather_radar;
  int32_t weather_radar;
} pyramid_data_model_agra_CapabilityTaxonomyType_c;

typedef struct pyramid_data_model_agra_CapabilityTaxonomyUniversalBaseType_c {
  pyramid_slice_t capability;
  uint8_t has_capability_details;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_details;
  pyramid_slice_t threat_capability;
  pyramid_slice_t capability_instance_id;
} pyramid_data_model_agra_CapabilityTaxonomyUniversalBaseType_c;

typedef struct pyramid_data_model_agra_ActivityByType_c {
  uint32_t activity;
  uint8_t has_activity_amplification;
  uint32_t activity_amplification;
  int32_t activity_category;
  uint8_t has_custom_activity;
  pyramid_data_model_agra_ForeignKeyType_c custom_activity;
  uint8_t has_activity_sub_category;
  int32_t activity_sub_category;
} pyramid_data_model_agra_ActivityByType_c;

typedef struct pyramid_data_model_agra_BehaviorType_c {
  pyramid_slice_t capability;  /* from CapabilityTaxonomyUniversalBaseType */
  uint8_t has_capability_details;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_details;  /* from CapabilityTaxonomyUniversalBaseType */
  pyramid_slice_t threat_capability;  /* from CapabilityTaxonomyUniversalBaseType */
  pyramid_slice_t capability_instance_id;  /* from CapabilityTaxonomyUniversalBaseType */
  pyramid_slice_t activity;
} pyramid_data_model_agra_BehaviorType_c;

typedef struct pyramid_data_model_agra_MA_EntityCharacteristicType_c {
  uint8_t has_identity;
  pyramid_data_model_agra_IdentityComparisonType_c identity;
  uint8_t has_identity_staleness;
  pyramid_str_t identity_staleness;
  uint8_t has_position_uncertainty;
  float position_uncertainty;
  uint8_t has_position_staleness;
  pyramid_str_t position_staleness;
  uint8_t has_prioritization_list;
  pyramid_data_model_agra_MA_PrioritizationListValueType_c prioritization_list;
  uint8_t has_behavior;
  pyramid_data_model_agra_BehaviorType_c behavior;
} pyramid_data_model_agra_MA_EntityCharacteristicType_c;

typedef struct pyramid_data_model_agra_MA_EntityComparativeType_c {
  pyramid_data_model_agra_MA_EntityCharacteristicType_c characteristic;
  int32_t comparator;
} pyramid_data_model_agra_MA_EntityComparativeType_c;

typedef struct pyramid_data_model_agra_MA_EntityFilterType_c {
  uint8_t has_designation;
  pyramid_data_model_agra_MA_DesignationFilterType_c designation;
  pyramid_slice_t characteristics;
  pyramid_slice_t entity_id;
  pyramid_slice_t source;
  pyramid_slice_t geo_filter;
  pyramid_slice_t orbital_filter;
} pyramid_data_model_agra_MA_EntityFilterType_c;

typedef struct pyramid_data_model_agra_MA_SystemCharacteristicType_c {
  uint8_t has_identity;
  pyramid_data_model_agra_EntityIdentityChoiceType_c identity;
  uint8_t has_position_uncertainty;
  float position_uncertainty;
  uint8_t has_position_staleness;
  pyramid_str_t position_staleness;
  uint8_t has_prioritization_list;
  pyramid_data_model_agra_MA_PrioritizationListValueType_c prioritization_list;
  uint8_t has_behavior;
  pyramid_data_model_agra_BehaviorType_c behavior;
} pyramid_data_model_agra_MA_SystemCharacteristicType_c;

typedef struct pyramid_data_model_agra_MA_SystemComparativeType_c {
  pyramid_data_model_agra_MA_SystemCharacteristicType_c characteristic;
  int32_t comparator;
} pyramid_data_model_agra_MA_SystemComparativeType_c;

typedef struct pyramid_data_model_agra_MA_SystemFilterType_c {
  pyramid_slice_t characteristics;
  pyramid_slice_t system_id;
  pyramid_slice_t geo_filter;
  pyramid_slice_t orbital_filter;
} pyramid_data_model_agra_MA_SystemFilterType_c;

typedef struct pyramid_data_model_agra_MA_AssetFilterType_c {
  uint8_t has_system;
  pyramid_data_model_agra_MA_SystemFilterType_c system;
  uint8_t has_entity;
  pyramid_data_model_agra_MA_EntityFilterType_c entity;
} pyramid_data_model_agra_MA_AssetFilterType_c;

typedef struct pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c {
  pyramid_slice_t capability;  /* from CapabilityTaxonomyUniversalBaseType */
  uint8_t has_capability_details;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_details;  /* from CapabilityTaxonomyUniversalBaseType */
  pyramid_slice_t threat_capability;  /* from CapabilityTaxonomyUniversalBaseType */
  pyramid_slice_t capability_instance_id;  /* from CapabilityTaxonomyUniversalBaseType */
  pyramid_slice_t threat;
} pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c;

typedef struct pyramid_data_model_agra_MA_AccessAssessmentFilterType_c {
  pyramid_slice_t assessment_name;
  uint8_t has_assessed_capabilities;
  pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c assessed_capabilities;
  uint8_t has_subject;
  pyramid_data_model_agra_MA_AssetFilterType_c subject;
  uint8_t has_assessed_object;
  pyramid_data_model_agra_MA_AssetFilterType_c assessed_object;
  pyramid_slice_t access_event;
} pyramid_data_model_agra_MA_AccessAssessmentFilterType_c;

typedef struct pyramid_data_model_agra_FileLocationID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_FileLocationID_Type_c;

typedef struct pyramid_data_model_agra_PercentRangeType_c {
  double minimum;
  double maximum;
} pyramid_data_model_agra_PercentRangeType_c;

typedef struct pyramid_data_model_agra_OpConstraintWeightingValueType_c {
  uint8_t has_discrete;
  double discrete;
  uint8_t has_range;
  pyramid_data_model_agra_PercentRangeType_c range;
  uint8_t has_likelihood;
  int32_t likelihood;
} pyramid_data_model_agra_OpConstraintWeightingValueType_c;

typedef struct pyramid_data_model_agra_OpConstraintWeightingType_c {
  int32_t constraint;
  int32_t constraint_type;
  pyramid_data_model_agra_OpConstraintWeightingValueType_c weighting;
  uint8_t has_amplification_id;
  pyramid_data_model_agra_FileLocationID_Type_c amplification_id;
} pyramid_data_model_agra_OpConstraintWeightingType_c;

typedef struct pyramid_data_model_agra_VulnerabilityLevelsType_c {
  pyramid_str_t composite_exposure_time;
  pyramid_str_t cumulative_exposure_time;
} pyramid_data_model_agra_VulnerabilityLevelsType_c;

typedef struct pyramid_data_model_agra_RiskSettingType_c {
  int32_t allowed_risk;
  uint8_t has_maximum_continuous_exposure;
  pyramid_data_model_agra_VulnerabilityLevelsType_c maximum_continuous_exposure;
} pyramid_data_model_agra_RiskSettingType_c;

typedef struct pyramid_data_model_agra_SurvivabilityRiskSettingType_c {
  int32_t risk_prioritization;
  pyramid_slice_t risk_level;
} pyramid_data_model_agra_SurvivabilityRiskSettingType_c;

typedef struct pyramid_data_model_agra_NormalizationTableType_c {
  float actual_metric_value;
  float normalized_metric_value;
} pyramid_data_model_agra_NormalizationTableType_c;

typedef struct pyramid_data_model_agra_RF_TaskNormalizedMetricsType_c {
  pyramid_slice_t capability_id;
  pyramid_data_model_agra_ForeignKeyType_c metric_identifier;
  int32_t interpolation_enum;
  pyramid_slice_t normalization_elements;
} pyramid_data_model_agra_RF_TaskNormalizedMetricsType_c;

typedef struct pyramid_data_model_agra_MetricValueType_c {
  pyramid_data_model_agra_ForeignKeyType_c metric_identifier;
  pyramid_str_t description_of_metric;
  uint8_t has_negotiable_options;
  int32_t negotiable_options;
  float weight;
  float value;
} pyramid_data_model_agra_MetricValueType_c;

typedef struct pyramid_data_model_agra_PerformanceMetricSetType_c {
  uint32_t performance_constraint_index;
  int32_t merit;
  uint8_t has_negotiable_options;
  int32_t negotiable_options;
  pyramid_slice_t performance_metric_set;
} pyramid_data_model_agra_PerformanceMetricSetType_c;

typedef struct pyramid_data_model_agra_RF_TaskPerformanceConstraintType_c {
  pyramid_slice_t performance_constraint;
  uint8_t has_capability_id;
  pyramid_data_model_agra_CapabilityID_Type_c capability_id;
} pyramid_data_model_agra_RF_TaskPerformanceConstraintType_c;

typedef struct pyramid_data_model_agra_RF_TaskPerformanceType_c {
  pyramid_slice_t performance_constraints;
  pyramid_slice_t normalized_metrics;
} pyramid_data_model_agra_RF_TaskPerformanceType_c;

typedef struct pyramid_data_model_agra_MA_AnalyticConstraintsType_c {
  pyramid_slice_t op_constraint;
  pyramid_slice_t risk_setting;
  uint8_t has_access_assessment_threshold;
  pyramid_data_model_agra_MA_AccessAssessmentFilterType_c access_assessment_threshold;
  uint8_t has_rf_task_performance;
  pyramid_data_model_agra_RF_TaskPerformanceType_c rf_task_performance;
} pyramid_data_model_agra_MA_AnalyticConstraintsType_c;

typedef struct pyramid_data_model_agra_MA_RequirementConstraintsType_c {
  uint8_t has_rank;
  pyramid_data_model_agra_ComparableRankingType_c rank;
  uint8_t has_interrupt_lower_rank;
  uint8_t interrupt_lower_rank;
  uint8_t has_allocation;
  pyramid_data_model_agra_MA_RequirementAllocationParametersType_c allocation;
  pyramid_slice_t timing;
  pyramid_slice_t kinematic;
  pyramid_slice_t dependency;
  uint8_t has_max_product_dissemination_classification_level;
  pyramid_data_model_agra_DataProductClassificationLevelType_c max_product_dissemination_classification_level;
  uint8_t has_analytic;
  pyramid_data_model_agra_MA_AnalyticConstraintsType_c analytic;
  uint8_t has_acceptable_classification_level;
  pyramid_data_model_agra_SecurityInformationType_c acceptable_classification_level;
  uint8_t has_comms_required;
  uint8_t comms_required;
  uint8_t has_allowed_requirement_types;
  pyramid_data_model_agra_MA_RequirementTaxonomyType_c allowed_requirement_types;
  uint8_t has_excluded_requirement_types;
  pyramid_data_model_agra_MA_RequirementTaxonomyType_c excluded_requirement_types;
  pyramid_str_t constraints_narrative;
  pyramid_slice_t allowed_domain_pairing;
  pyramid_slice_t excluded_domain_pairing;
  uint8_t has_shot_doctrine_parameters;
  pyramid_data_model_agra_MA_ShotDoctrineParametersType_c shot_doctrine_parameters;
} pyramid_data_model_agra_MA_RequirementConstraintsType_c;

typedef struct pyramid_data_model_agra_RequirementTaxonomyType_c {
  pyramid_slice_t effect;
  pyramid_slice_t action;
  pyramid_slice_t task;
  pyramid_slice_t capability_command;
  pyramid_slice_t response;
} pyramid_data_model_agra_RequirementTaxonomyType_c;

typedef struct pyramid_data_model_agra_PrioritizationType_c {
  pyramid_data_model_agra_ComparableRankingType_c priority_value;
  uint8_t has_requirement_type;
  pyramid_data_model_agra_RequirementTaxonomyType_c requirement_type;
} pyramid_data_model_agra_PrioritizationType_c;

typedef struct pyramid_data_model_agra_PrioritizationListValueType_c {
  int32_t list_type;
  pyramid_data_model_agra_PrioritizationType_c prioritization;
} pyramid_data_model_agra_PrioritizationListValueType_c;

typedef struct pyramid_data_model_agra_EntityCharacteristicType_c {
  uint8_t has_identity;
  pyramid_data_model_agra_IdentityComparisonType_c identity;
  uint8_t has_identity_staleness;
  pyramid_str_t identity_staleness;
  uint8_t has_position_uncertainty;
  float position_uncertainty;
  uint8_t has_position_staleness;
  pyramid_str_t position_staleness;
  uint8_t has_prioritization_list;
  pyramid_data_model_agra_PrioritizationListValueType_c prioritization_list;
  uint8_t has_behavior;
  pyramid_data_model_agra_BehaviorType_c behavior;
} pyramid_data_model_agra_EntityCharacteristicType_c;

typedef struct pyramid_data_model_agra_EntityComparativeType_c {
  pyramid_data_model_agra_EntityCharacteristicType_c characteristic;
  int32_t comparator;
} pyramid_data_model_agra_EntityComparativeType_c;

typedef struct pyramid_data_model_agra_DesignationFilterType_c {
  pyramid_slice_t designation_type;
  uint8_t has_designation_objective;
  pyramid_data_model_agra_RequirementTaxonomyType_c designation_objective;
} pyramid_data_model_agra_DesignationFilterType_c;

typedef struct pyramid_data_model_agra_EntityFilterType_c {
  uint8_t has_designation;
  pyramid_data_model_agra_DesignationFilterType_c designation;
  pyramid_slice_t characteristics;
  pyramid_slice_t entity_id;
  pyramid_slice_t source;
  pyramid_slice_t geo_filter;
  pyramid_slice_t orbital_filter;
} pyramid_data_model_agra_EntityFilterType_c;

typedef struct pyramid_data_model_agra_SystemCharacteristicType_c {
  uint8_t has_identity;
  pyramid_data_model_agra_EntityIdentityChoiceType_c identity;
  uint8_t has_position_uncertainty;
  float position_uncertainty;
  uint8_t has_position_staleness;
  pyramid_str_t position_staleness;
  uint8_t has_prioritization_list;
  pyramid_data_model_agra_PrioritizationListValueType_c prioritization_list;
  uint8_t has_behavior;
  pyramid_data_model_agra_BehaviorType_c behavior;
} pyramid_data_model_agra_SystemCharacteristicType_c;

typedef struct pyramid_data_model_agra_SystemComparativeType_c {
  pyramid_data_model_agra_SystemCharacteristicType_c characteristic;
  int32_t comparator;
} pyramid_data_model_agra_SystemComparativeType_c;

typedef struct pyramid_data_model_agra_SystemFilterType_c {
  pyramid_slice_t characteristics;
  pyramid_slice_t system_id;
  pyramid_slice_t geo_filter;
  pyramid_slice_t orbital_filter;
} pyramid_data_model_agra_SystemFilterType_c;

typedef struct pyramid_data_model_agra_AssetFilterType_c {
  uint8_t has_system;
  pyramid_data_model_agra_SystemFilterType_c system;
  uint8_t has_entity;
  pyramid_data_model_agra_EntityFilterType_c entity;
} pyramid_data_model_agra_AssetFilterType_c;

typedef struct pyramid_data_model_agra_AccessAssessmentFilterType_c {
  pyramid_slice_t assessment_name;
  uint8_t has_assessed_capabilities;
  pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c assessed_capabilities;
  uint8_t has_subject;
  pyramid_data_model_agra_AssetFilterType_c subject;
  uint8_t has_assessed_object;
  pyramid_data_model_agra_AssetFilterType_c assessed_object;
  pyramid_slice_t access_event;
} pyramid_data_model_agra_AccessAssessmentFilterType_c;

typedef struct pyramid_data_model_agra_AnalyticConstraintsType_c {
  pyramid_slice_t op_constraint;
  pyramid_slice_t risk_setting;
  uint8_t has_access_assessment_threshold;
  pyramid_data_model_agra_AccessAssessmentFilterType_c access_assessment_threshold;
  uint8_t has_rf_task_performance;
  pyramid_data_model_agra_RF_TaskPerformanceType_c rf_task_performance;
} pyramid_data_model_agra_AnalyticConstraintsType_c;

typedef struct pyramid_data_model_agra_RequirementTargetConstraintsType_c {
  uint8_t has_behavior;
  pyramid_data_model_agra_BehaviorType_c behavior;
  uint8_t has_behavior_impact;
  pyramid_data_model_agra_AnalyticConstraintsType_c behavior_impact;
} pyramid_data_model_agra_RequirementTargetConstraintsType_c;

typedef struct pyramid_data_model_agra_MA_BanzaiType_c {
  int32_t termination_type;
} pyramid_data_model_agra_MA_BanzaiType_c;

typedef struct pyramid_data_model_agra_MA_SkateType_c {
  double maneuver_out_range;
} pyramid_data_model_agra_MA_SkateType_c;

typedef struct pyramid_data_model_agra_MA_InterceptTacticType_c {
  uint8_t has_skate_type;
  pyramid_data_model_agra_MA_SkateType_c skate_type;
  uint8_t has_banzai_type;
  pyramid_data_model_agra_MA_BanzaiType_c banzai_type;
} pyramid_data_model_agra_MA_InterceptTacticType_c;

typedef struct pyramid_data_model_agra_MA_EngagementParametersType_c {
  uint8_t has_intercept_tactic;
  pyramid_data_model_agra_MA_InterceptTacticType_c intercept_tactic;
} pyramid_data_model_agra_MA_EngagementParametersType_c;

typedef struct pyramid_data_model_agra_MA_AttackKineticParametersType_c {
  uint8_t has_engagement_parameters;
  pyramid_data_model_agra_MA_EngagementParametersType_c engagement_parameters;
} pyramid_data_model_agra_MA_AttackKineticParametersType_c;

typedef struct pyramid_data_model_agra_MA_UniqueActionDataType_c {
  uint8_t has_attack_kinetic_parameters;
  pyramid_data_model_agra_MA_AttackKineticParametersType_c attack_kinetic_parameters;
} pyramid_data_model_agra_MA_UniqueActionDataType_c;

typedef struct pyramid_data_model_agra_RemarksType_c {
  pyramid_str_t display_name;
  pyramid_str_t detail;
} pyramid_data_model_agra_RemarksType_c;

typedef struct pyramid_data_model_agra_AOCO_TraceabilityType_c {
  uint32_t scm_identifier;
} pyramid_data_model_agra_AOCO_TraceabilityType_c;

typedef struct pyramid_data_model_agra_CS_STO_TraceabilityType_c {
  pyramid_str_t cycle_number;
  uint32_t version;
} pyramid_data_model_agra_CS_STO_TraceabilityType_c;

typedef struct pyramid_data_model_agra_CollectionDeckTraceabilityType_c {
  pyramid_str_t collection_deck_mission_identifier;
  pyramid_str_t collection_deck_task_identifier;
} pyramid_data_model_agra_CollectionDeckTraceabilityType_c;

typedef struct pyramid_data_model_agra_ATO_TraceabilityType_c {
  pyramid_str_t mission_call_sign;
  pyramid_str_t mission_identifier;
} pyramid_data_model_agra_ATO_TraceabilityType_c;

typedef struct pyramid_data_model_agra_ACTDF_CollectionPlanType_c {
  pyramid_data_model_agra_ForeignKeyType_c plan_key;
  uint8_t has_version;
  double version;
} pyramid_data_model_agra_ACTDF_CollectionPlanType_c;

typedef struct pyramid_data_model_agra_ACTDF_TaskID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ACTDF_TaskID_Type_c;

typedef struct pyramid_data_model_agra_ACTDF_TraceabilityType_c {
  pyramid_str_t actdf_mission_identifier;
  pyramid_data_model_agra_ACTDF_CollectionPlanType_c actdf_collection_plan_identifier;
  pyramid_data_model_agra_ACTDF_TaskID_Type_c actdf_task_id;
} pyramid_data_model_agra_ACTDF_TraceabilityType_c;

typedef struct pyramid_data_model_agra_EEI_ID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_EEI_ID_Type_c;

typedef struct pyramid_data_model_agra_TraceabilityType_c {
  pyramid_slice_t requirement;
  pyramid_slice_t collection_deck_traceability;
  pyramid_slice_t actdf_traceability;
  pyramid_slice_t eei_id;
  pyramid_slice_t ato_traceability;
  uint8_t has_aoco_traceability;
  pyramid_data_model_agra_AOCO_TraceabilityType_c aoco_traceability;
  uint8_t has_sto_traceability;
  pyramid_data_model_agra_CS_STO_TraceabilityType_c sto_traceability;
} pyramid_data_model_agra_TraceabilityType_c;

typedef struct pyramid_data_model_agra_RequirementMetadataType_c {
  uint8_t has_traceability;
  pyramid_data_model_agra_TraceabilityType_c traceability;
  uint8_t has_source;
  int32_t source;
  uint8_t has_narrative;
  pyramid_data_model_agra_RemarksType_c narrative;
} pyramid_data_model_agra_RequirementMetadataType_c;

typedef struct pyramid_data_model_agra_RequirementGuidanceType_c {
  pyramid_slice_t usage;
  uint8_t has_tactical_planning_and_execution;
  int32_t tactical_planning_and_execution;
  uint8_t has_failure;
  int32_t failure;
} pyramid_data_model_agra_RequirementGuidanceType_c;

typedef struct pyramid_data_model_agra_PointTargetType_c {
  pyramid_data_model_agra_PointChoiceType_c point;
  uint8_t has_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c velocity;
  uint8_t has_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c uncertainty;
} pyramid_data_model_agra_PointTargetType_c;

typedef struct pyramid_data_model_agra_OpZoneID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OpZoneID_Type_c;

typedef struct pyramid_data_model_agra_OpVolumeID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OpVolumeID_Type_c;

typedef struct pyramid_data_model_agra_ZoneExternalType_c {
  pyramid_data_model_agra_AreaChoiceType_c shape;  /* from ZoneType */
  uint8_t has_min_altitude;
  double min_altitude;  /* from ZoneType */
  uint8_t has_max_altitude;
  double max_altitude;  /* from ZoneType */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from ZoneType */
  pyramid_str_t dsa_identifier;
} pyramid_data_model_agra_ZoneExternalType_c;

typedef struct pyramid_data_model_agra_LineRelativeType_c {
  pyramid_data_model_agra_ReferenceFrameID_Type_c reference_frame_id;
  pyramid_slice_t position_offset;
} pyramid_data_model_agra_LineRelativeType_c;

typedef struct pyramid_data_model_agra_LinePoint2D_Type_c {
  double latitude;  /* from Point2D_Type */
  double longitude;  /* from Point2D_Type */
  uint8_t has_altitude;
  double altitude;  /* from Point2D_Type */
  uint8_t has_altitude_range;
  pyramid_data_model_agra_AltitudeRangeType_c altitude_range;  /* from Point2D_Type */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from Point2D_Type */
  pyramid_str_t timestamp;  /* from Point2D_Type */
  uint8_t has_left_width;
  double left_width;
  uint8_t has_right_width;
  double right_width;
} pyramid_data_model_agra_LinePoint2D_Type_c;

typedef struct pyramid_data_model_agra_LinePointChoiceType_Point_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_LinePointChoiceType_Point_List_c;

typedef struct pyramid_data_model_agra_LinePointChoiceType_c {
  uint8_t has_point;
  pyramid_data_model_agra_LinePointChoiceType_Point_List_c point;
  uint8_t has_relative_point;
  pyramid_data_model_agra_LineRelativeType_c relative_point;
} pyramid_data_model_agra_LinePointChoiceType_c;

typedef struct pyramid_data_model_agra_LineType_c {
  pyramid_data_model_agra_LinePointChoiceType_c point_choice;
  int32_t line_projection;
  uint8_t has_left_width;
  double left_width;
  uint8_t has_right_width;
  double right_width;
  uint8_t has_min_altitude;
  double min_altitude;
  uint8_t has_max_altitude;
  double max_altitude;
  uint8_t has_altitude_reference;
  int32_t altitude_reference;
} pyramid_data_model_agra_LineType_c;

typedef struct pyramid_data_model_agra_LineTargetType_c {
  pyramid_data_model_agra_LinePointChoiceType_c point_choice;  /* from LineType */
  int32_t line_projection;  /* from LineType */
  uint8_t has_left_width;
  double left_width;  /* from LineType */
  uint8_t has_right_width;
  double right_width;  /* from LineType */
  uint8_t has_min_altitude;
  double min_altitude;  /* from LineType */
  uint8_t has_max_altitude;
  double max_altitude;  /* from LineType */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from LineType */
  pyramid_str_t loc_identifier;
} pyramid_data_model_agra_LineTargetType_c;

typedef struct pyramid_data_model_agra_OpLineID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OpLineID_Type_c;

typedef struct pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c;

typedef struct pyramid_data_model_agra_SignalID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SignalID_Type_c;

typedef struct pyramid_data_model_agra_TargetType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_operator_location_of_interest_id;
  pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c operator_location_of_interest_id;
  uint8_t has_signal_id;
  pyramid_data_model_agra_SignalID_Type_c signal_id;
  uint8_t has_op_point_id;
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
  uint8_t has_op_zone_id;
  pyramid_data_model_agra_OpZoneID_Type_c op_zone_id;
  uint8_t has_op_volume_id;
  pyramid_data_model_agra_OpVolumeID_Type_c op_volume_id;
  uint8_t has_op_line_id;
  pyramid_data_model_agra_OpLineID_Type_c op_line_id;
  uint8_t has_point_target;
  pyramid_data_model_agra_PointTargetType_c point_target;
  uint8_t has_zone_target;
  pyramid_data_model_agra_ZoneExternalType_c zone_target;
  uint8_t has_volume_target;
  pyramid_data_model_agra_OpVolumeType_c volume_target;
  uint8_t has_line_target;
  pyramid_data_model_agra_LineTargetType_c line_target;
} pyramid_data_model_agra_TargetType_c;

typedef struct pyramid_data_model_agra_IdentityKindInstanceType_c {
  uint8_t has_by_instance;
  pyramid_data_model_agra_TargetType_c by_instance;
  uint8_t has_by_identity;
  pyramid_data_model_agra_IdentityType_c by_identity;
} pyramid_data_model_agra_IdentityKindInstanceType_c;

typedef struct pyramid_data_model_agra_MA_ActionMDT_c {
  pyramid_data_model_agra_ActionID_Type_c action_id;
  int32_t action_type;
  uint8_t has_action_constraints;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c action_constraints;
  uint8_t has_action_guidance;
  pyramid_data_model_agra_RequirementGuidanceType_c action_guidance;
  uint8_t has_category_unique_data;
  pyramid_data_model_agra_MA_UniqueActionDataType_c category_unique_data;
  pyramid_slice_t target_object;
  uint8_t has_target_object_constraints;
  pyramid_data_model_agra_RequirementTargetConstraintsType_c target_object_constraints;
  pyramid_slice_t secondary_object;
  uint8_t has_metadata;
  pyramid_data_model_agra_RequirementMetadataType_c metadata;
} pyramid_data_model_agra_MA_ActionMDT_c;

typedef struct pyramid_data_model_agra_MA_ActionMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MA_ActionMDT_c message_data;
} pyramid_data_model_agra_MA_ActionMT_c;

typedef struct pyramid_data_model_agra_ExpendableType_c {
  uint32_t quantity;
  pyramid_data_model_agra_ForeignKeyType_c type;
} pyramid_data_model_agra_ExpendableType_c;

typedef struct pyramid_data_model_agra_BoundaryType_c {
  uint8_t has_polygon;
  pyramid_data_model_agra_PolygonType_c polygon;
  uint8_t has_ellipse;
  pyramid_data_model_agra_LocatedEllipseType_c ellipse;
} pyramid_data_model_agra_BoundaryType_c;

typedef struct pyramid_data_model_agra_EnduranceFootprintType_c {
  pyramid_data_model_agra_BoundaryType_c boundary;
  uint8_t has_reference_altitude;
  double reference_altitude;
  pyramid_str_t duration;
} pyramid_data_model_agra_EnduranceFootprintType_c;

typedef struct pyramid_data_model_agra_EnduranceBaseType_c {
  uint8_t has_fuel;
  double fuel;
  pyramid_str_t duration;
  pyramid_str_t duration_end;
  uint8_t has_percent;
  double percent;
} pyramid_data_model_agra_EnduranceBaseType_c;

typedef struct pyramid_data_model_agra_EnduranceType_c {
  uint8_t has_fuel;
  double fuel;  /* from EnduranceBaseType */
  pyramid_str_t duration;  /* from EnduranceBaseType */
  pyramid_str_t duration_end;  /* from EnduranceBaseType */
  uint8_t has_percent;
  double percent;  /* from EnduranceBaseType */
  uint8_t has_footprint;
  pyramid_data_model_agra_EnduranceFootprintType_c footprint;
} pyramid_data_model_agra_EnduranceType_c;

typedef struct pyramid_data_model_agra_WeaponStoreType_c {
  uint32_t quantity;
  pyramid_data_model_agra_StoreType_c weapon;
} pyramid_data_model_agra_WeaponStoreType_c;

typedef struct pyramid_data_model_agra_MetricsType_c {
  uint8_t has_endurance_usage;
  pyramid_data_model_agra_EnduranceType_c endurance_usage;
  pyramid_slice_t expendables;
  pyramid_slice_t weapons;
} pyramid_data_model_agra_MetricsType_c;

typedef struct pyramid_data_model_agra_MA_PackageSystemType_c {
  uint8_t has_all;
  pyramid_str_t all;
  uint8_t has_non;
  pyramid_str_t non;
  uint8_t has_package_id;
  pyramid_data_model_agra_PackageID_Type_c package_id;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
} pyramid_data_model_agra_MA_PackageSystemType_c;

typedef struct pyramid_data_model_agra_CannotComplyType_c {
  int32_t reason;
  pyramid_str_t description;
  uint8_t has_associated_id;
  pyramid_data_model_agra_ID_Type_c associated_id;
} pyramid_data_model_agra_CannotComplyType_c;

typedef struct pyramid_data_model_agra_ActivityID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ActivityID_Type_c;

typedef struct pyramid_data_model_agra_PlannedActivityID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_PlannedActivityID_Type_c;

typedef struct pyramid_data_model_agra_RoutePlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_RoutePlanID_Type_c;

typedef struct pyramid_data_model_agra_TaskPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_TaskPlanID_Type_c;

typedef struct pyramid_data_model_agra_ActivityPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_ActivityPlanID_Type_c;

typedef struct pyramid_data_model_agra_EffectPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_EffectPlanID_Type_c;

typedef struct pyramid_data_model_agra_OrbitActivityPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_OrbitActivityPlanID_Type_c;

typedef struct pyramid_data_model_agra_ResponsePlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_ResponsePlanID_Type_c;

typedef struct pyramid_data_model_agra_OrbitPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_OrbitPlanID_Type_c;

typedef struct pyramid_data_model_agra_ActionPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_ActionPlanID_Type_c;

typedef struct pyramid_data_model_agra_RouteActivityPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_RouteActivityPlanID_Type_c;

typedef struct pyramid_data_model_agra_MA_OpPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_MA_OpPlanID_Type_c;

typedef struct pyramid_data_model_agra_MA_PlansReferenceBaseType_c {
  pyramid_slice_t task_plan_id;
  pyramid_slice_t orbit_plan_id;
  pyramid_slice_t orbit_activity_plan_id;
  pyramid_slice_t route_plan_id;
  pyramid_slice_t route_activity_plan_id;
  pyramid_slice_t activity_plan_id;
  pyramid_slice_t effect_plan_id;
  pyramid_slice_t action_plan_id;
  pyramid_slice_t response_plan_id;
  pyramid_slice_t op_plan_id;
} pyramid_data_model_agra_MA_PlansReferenceBaseType_c;

typedef struct pyramid_data_model_agra_MissionPlanID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_MissionPlanID_Type_c;

typedef struct pyramid_data_model_agra_MA_PlansReferenceType_c {
  pyramid_slice_t task_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t orbit_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t orbit_activity_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t route_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t route_activity_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t activity_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t effect_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t action_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t response_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t op_plan_id;  /* from MA_PlansReferenceBaseType */
  pyramid_slice_t mission_plan_id;
} pyramid_data_model_agra_MA_PlansReferenceType_c;

typedef struct pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c {
  uint8_t has_plans;
  pyramid_data_model_agra_MA_PlansReferenceType_c plans;
  pyramid_slice_t planned_activity_id;
  pyramid_slice_t capability_id;
  pyramid_slice_t command_id;
  pyramid_slice_t activity_id;
} pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c;

typedef struct pyramid_data_model_agra_MA_RequirementExecutionStatusDetailsType_c {
  pyramid_data_model_agra_MA_PackageSystemType_c executing_system_or_package;
  int32_t execution_state;
  uint8_t has_execution_state_reason;
  pyramid_data_model_agra_CannotComplyType_c execution_state_reason;
  pyramid_slice_t actual_timing;
  uint8_t has_percent_completed;
  double percent_completed;
  uint8_t has_traceability;
  pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c traceability;
  uint8_t has_metrics;
  pyramid_data_model_agra_MetricsType_c metrics;
} pyramid_data_model_agra_MA_RequirementExecutionStatusDetailsType_c;

typedef struct pyramid_data_model_agra_MA_ActionStatusMDT_c {
  pyramid_data_model_agra_MA_PackageSystemType_c executing_system_or_package;  /* from MA_RequirementExecutionStatusDetailsType */
  int32_t execution_state;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_execution_state_reason;
  pyramid_data_model_agra_CannotComplyType_c execution_state_reason;  /* from MA_RequirementExecutionStatusDetailsType */
  pyramid_slice_t actual_timing;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_percent_completed;
  double percent_completed;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_traceability;
  pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c traceability;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_metrics;
  pyramid_data_model_agra_MetricsType_c metrics;  /* from MA_RequirementExecutionStatusDetailsType */
  pyramid_data_model_agra_ActionID_Type_c action_id;
} pyramid_data_model_agra_MA_ActionStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_ActionStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_ActionStatusMDT_c message_data;
} pyramid_data_model_agra_MA_ActionStatusMT_c;

typedef struct pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c;

typedef struct pyramid_data_model_agra_ScheduleType_TimeSpan_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_ScheduleType_TimeSpan_List_c;

typedef struct pyramid_data_model_agra_ScheduleType_c {
  uint8_t has_time_span;
  pyramid_data_model_agra_ScheduleType_TimeSpan_List_c time_span;
  uint8_t has_weekday_interval;
  pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c weekday_interval;
} pyramid_data_model_agra_ScheduleType_c;

typedef struct pyramid_data_model_agra_TimedZoneType_c {
  pyramid_data_model_agra_AreaChoiceType_c shape;  /* from ZoneType */
  uint8_t has_min_altitude;
  double min_altitude;  /* from ZoneType */
  uint8_t has_max_altitude;
  double max_altitude;  /* from ZoneType */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from ZoneType */
  pyramid_data_model_agra_ScheduleType_c schedule;
} pyramid_data_model_agra_TimedZoneType_c;

typedef struct pyramid_data_model_agra_DataRecordInstanceID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_DataRecordInstanceID_Type_c;

typedef struct pyramid_data_model_agra_DataRecordBaseType_c {
  uint8_t has_data_record_instance_id;
  pyramid_data_model_agra_DataRecordInstanceID_Type_c data_record_instance_id;
} pyramid_data_model_agra_DataRecordBaseType_c;

typedef struct pyramid_data_model_agra_RankCompareType_c {
  pyramid_data_model_agra_ComparableRankingType_c rank_value;
  int32_t comparator;
} pyramid_data_model_agra_RankCompareType_c;

typedef struct pyramid_data_model_agra_RequirementTaxonomyDetailedType_c {
  pyramid_slice_t effect;  /* from RequirementTaxonomyType */
  pyramid_slice_t action;  /* from RequirementTaxonomyType */
  pyramid_slice_t task;  /* from RequirementTaxonomyType */
  pyramid_slice_t capability_command;  /* from RequirementTaxonomyType */
  pyramid_slice_t response;  /* from RequirementTaxonomyType */
  uint8_t has_capability_taxonomy;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_taxonomy;
} pyramid_data_model_agra_RequirementTaxonomyDetailedType_c;

typedef struct pyramid_data_model_agra_RequirementTriggerType_c {
  uint8_t has_rank;
  pyramid_data_model_agra_RankCompareType_c rank;
  uint8_t has_requirement_types;
  pyramid_data_model_agra_RequirementTaxonomyDetailedType_c requirement_types;
} pyramid_data_model_agra_RequirementTriggerType_c;

typedef struct pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c;

typedef struct pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c {
  uint8_t has_acquisition_cost_levels;
  pyramid_data_model_agra_VulnerabilityLevelsType_c acquisition_cost_levels;
  uint8_t has_track_cost_levels;
  pyramid_data_model_agra_VulnerabilityLevelsType_c track_cost_levels;
  uint8_t has_intercept_cost_levels;
  pyramid_data_model_agra_VulnerabilityLevelsType_c intercept_cost_levels;
  uint8_t has_launch_cost_levels;
  pyramid_data_model_agra_VulnerabilityLevelsType_c launch_cost_levels;
  uint8_t has_probability_of_survival;
  double probability_of_survival;
} pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c;

typedef struct pyramid_data_model_agra_ThresholdVulnerabilityType_c {
  uint8_t has_vulnerability_without_suppression;
  pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c vulnerability_without_suppression;
  uint8_t has_vulnerability_with_suppression;
  pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c vulnerability_with_suppression;
  uint8_t has_exposure_event_probability_threshold;
  double exposure_event_probability_threshold;
} pyramid_data_model_agra_ThresholdVulnerabilityType_c;

typedef struct pyramid_data_model_agra_ThreatVulnerabilityByCapabilityType_c {
  int32_t threat_capability;
  pyramid_data_model_agra_ThresholdVulnerabilityType_c threshold;
} pyramid_data_model_agra_ThreatVulnerabilityByCapabilityType_c;

typedef struct pyramid_data_model_agra_PlanVulnerabilityType_c {
  pyramid_slice_t vulnerability_by_capability;
  uint8_t has_comparator;
  int32_t comparator;
} pyramid_data_model_agra_PlanVulnerabilityType_c;

typedef struct pyramid_data_model_agra_PlanningByResultTriggerType_c {
  uint8_t has_replan_required;
  pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c replan_required;
  uint8_t has_vulnerability_changed;
  pyramid_data_model_agra_PlanVulnerabilityType_c vulnerability_changed;
  uint8_t has_requirement_unallocated;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_unallocated;
} pyramid_data_model_agra_PlanningByResultTriggerType_c;

typedef struct pyramid_data_model_agra_DefaultResponseType_c {
  int32_t approval_type;
  pyramid_str_t timeout;
} pyramid_data_model_agra_DefaultResponseType_c;

typedef struct pyramid_data_model_agra_ApprovalPolicyBaseType_c {
  int32_t policy;
  uint8_t has_timeout_policy;
  pyramid_data_model_agra_DefaultResponseType_c timeout_policy;
} pyramid_data_model_agra_ApprovalPolicyBaseType_c;

typedef struct pyramid_data_model_agra_ByResultPolicyType_c {
  int32_t policy;  /* from ApprovalPolicyBaseType */
  uint8_t has_timeout_policy;
  pyramid_data_model_agra_DefaultResponseType_c timeout_policy;  /* from ApprovalPolicyBaseType */
  pyramid_data_model_agra_PlanningByResultTriggerType_c result;
} pyramid_data_model_agra_ByResultPolicyType_c;

typedef struct pyramid_data_model_agra_RoutePlanPartsType_c {
  pyramid_slice_t route_type;
} pyramid_data_model_agra_RoutePlanPartsType_c;

typedef struct pyramid_data_model_agra_EffectPlanPartsType_c {
  pyramid_slice_t effect_plan;
} pyramid_data_model_agra_EffectPlanPartsType_c;

typedef struct pyramid_data_model_agra_ActionPlanPartsType_c {
  pyramid_slice_t action_plan;
} pyramid_data_model_agra_ActionPlanPartsType_c;

typedef struct pyramid_data_model_agra_OpPointCategoriesType_c {
  uint8_t has_general;
  int32_t general;
  uint8_t has_hazard;
  int32_t hazard;
  uint8_t has_reference;
  int32_t reference;
  uint8_t has_station;
  int32_t station;
  uint8_t has_emergency;
  int32_t emergency;
} pyramid_data_model_agra_OpPointCategoriesType_c;

typedef struct pyramid_data_model_agra_MA_OpPlanPartsType_c {
  pyramid_slice_t op_point_category;
  pyramid_slice_t op_line_category;
  pyramid_slice_t op_zone_category;
  pyramid_slice_t op_volume_category;
  pyramid_str_t op_routing;
} pyramid_data_model_agra_MA_OpPlanPartsType_c;

typedef struct pyramid_data_model_agra_MA_TaskPlanPartsType_c {
  pyramid_slice_t task_type;
} pyramid_data_model_agra_MA_TaskPlanPartsType_c;

typedef struct pyramid_data_model_agra_ResponsePlanPartsType_c {
  pyramid_slice_t response_type;
} pyramid_data_model_agra_ResponsePlanPartsType_c;

typedef struct pyramid_data_model_agra_MA_PlanPartsBaseType_c {
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanPartsType_c effect_plan;
  uint8_t has_action_plan;
  pyramid_data_model_agra_ActionPlanPartsType_c action_plan;
  uint8_t has_task_plan;
  pyramid_data_model_agra_MA_TaskPlanPartsType_c task_plan;
  uint8_t has_response_plan;
  pyramid_data_model_agra_ResponsePlanPartsType_c response_plan;
  uint8_t has_op_plan;
  pyramid_data_model_agra_MA_OpPlanPartsType_c op_plan;
} pyramid_data_model_agra_MA_PlanPartsBaseType_c;

typedef struct pyramid_data_model_agra_MA_ActivityPlanPartsType_c {
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanPartsType_c effect_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_action_plan;
  pyramid_data_model_agra_ActionPlanPartsType_c action_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_task_plan;
  pyramid_data_model_agra_MA_TaskPlanPartsType_c task_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_response_plan;
  pyramid_data_model_agra_ResponsePlanPartsType_c response_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_op_plan;
  pyramid_data_model_agra_MA_OpPlanPartsType_c op_plan;  /* from MA_PlanPartsBaseType */
  pyramid_slice_t capability_command;
  pyramid_slice_t supporting_capability_command;
  pyramid_str_t vehicle_settings;
  pyramid_str_t comms_usage;
  pyramid_str_t product_tasks;
} pyramid_data_model_agra_MA_ActivityPlanPartsType_c;

typedef struct pyramid_data_model_agra_OrbitPlanPartsType_c {
  pyramid_slice_t orbit_part;
  pyramid_slice_t kinematics_type;
} pyramid_data_model_agra_OrbitPlanPartsType_c;

typedef struct pyramid_data_model_agra_MA_PlanPartsType_c {
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanPartsType_c effect_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_action_plan;
  pyramid_data_model_agra_ActionPlanPartsType_c action_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_task_plan;
  pyramid_data_model_agra_MA_TaskPlanPartsType_c task_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_response_plan;
  pyramid_data_model_agra_ResponsePlanPartsType_c response_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_op_plan;
  pyramid_data_model_agra_MA_OpPlanPartsType_c op_plan;  /* from MA_PlanPartsBaseType */
  uint8_t has_activity_plan;
  pyramid_data_model_agra_MA_ActivityPlanPartsType_c activity_plan;
  uint8_t has_route_plan;
  pyramid_data_model_agra_RoutePlanPartsType_c route_plan;
  uint8_t has_route_activity_plan;
  pyramid_data_model_agra_MA_ActivityPlanPartsType_c route_activity_plan;
  uint8_t has_orbit_plan;
  pyramid_data_model_agra_OrbitPlanPartsType_c orbit_plan;
  uint8_t has_orbit_activity_plan;
  pyramid_data_model_agra_MA_ActivityPlanPartsType_c orbit_activity_plan;
} pyramid_data_model_agra_MA_PlanPartsType_c;

typedef struct pyramid_data_model_agra_CommAllocationPartsType_c {
  pyramid_slice_t comm_capability;
} pyramid_data_model_agra_CommAllocationPartsType_c;

typedef struct pyramid_data_model_agra_TaskPlanPartsType_c {
  pyramid_slice_t task_type;
} pyramid_data_model_agra_TaskPlanPartsType_c;

typedef struct pyramid_data_model_agra_PlanPartsBaseType_c {
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanPartsType_c effect_plan;
  uint8_t has_action_plan;
  pyramid_data_model_agra_ActionPlanPartsType_c action_plan;
  uint8_t has_task_plan;
  pyramid_data_model_agra_TaskPlanPartsType_c task_plan;
  uint8_t has_response_plan;
  pyramid_data_model_agra_ResponsePlanPartsType_c response_plan;
} pyramid_data_model_agra_PlanPartsBaseType_c;

typedef struct pyramid_data_model_agra_ActivityPlanPartsType_c {
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanPartsType_c effect_plan;  /* from PlanPartsBaseType */
  uint8_t has_action_plan;
  pyramid_data_model_agra_ActionPlanPartsType_c action_plan;  /* from PlanPartsBaseType */
  uint8_t has_task_plan;
  pyramid_data_model_agra_TaskPlanPartsType_c task_plan;  /* from PlanPartsBaseType */
  uint8_t has_response_plan;
  pyramid_data_model_agra_ResponsePlanPartsType_c response_plan;  /* from PlanPartsBaseType */
  pyramid_slice_t capability_command;
  pyramid_slice_t supporting_capability_command;
  pyramid_str_t vehicle_settings;
  pyramid_str_t comms_usage;
  pyramid_str_t product_tasks;
} pyramid_data_model_agra_ActivityPlanPartsType_c;

typedef struct pyramid_data_model_agra_PlanPartsType_c {
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanPartsType_c effect_plan;  /* from PlanPartsBaseType */
  uint8_t has_action_plan;
  pyramid_data_model_agra_ActionPlanPartsType_c action_plan;  /* from PlanPartsBaseType */
  uint8_t has_task_plan;
  pyramid_data_model_agra_TaskPlanPartsType_c task_plan;  /* from PlanPartsBaseType */
  uint8_t has_response_plan;
  pyramid_data_model_agra_ResponsePlanPartsType_c response_plan;  /* from PlanPartsBaseType */
  uint8_t has_activity_plan;
  pyramid_data_model_agra_ActivityPlanPartsType_c activity_plan;
  uint8_t has_route_plan;
  pyramid_data_model_agra_RoutePlanPartsType_c route_plan;
  uint8_t has_route_activity_plan;
  pyramid_data_model_agra_ActivityPlanPartsType_c route_activity_plan;
  uint8_t has_orbit_plan;
  pyramid_data_model_agra_OrbitPlanPartsType_c orbit_plan;
  uint8_t has_orbit_activity_plan;
  pyramid_data_model_agra_ActivityPlanPartsType_c orbit_activity_plan;
} pyramid_data_model_agra_PlanPartsType_c;

typedef struct pyramid_data_model_agra_ConstrainingPlanPartsType_c {
  pyramid_data_model_agra_PlanPartsBaseType_c base;  /* from PlanPartsType */
  uint8_t has_activity_plan;
  pyramid_data_model_agra_ActivityPlanPartsType_c activity_plan;  /* from PlanPartsType */
  uint8_t has_route_plan;
  pyramid_data_model_agra_RoutePlanPartsType_c route_plan;  /* from PlanPartsType */
  uint8_t has_route_activity_plan;
  pyramid_data_model_agra_ActivityPlanPartsType_c route_activity_plan;  /* from PlanPartsType */
  uint8_t has_orbit_plan;
  pyramid_data_model_agra_OrbitPlanPartsType_c orbit_plan;  /* from PlanPartsType */
  uint8_t has_orbit_activity_plan;
  pyramid_data_model_agra_ActivityPlanPartsType_c orbit_activity_plan;  /* from PlanPartsType */
  uint8_t has_comm_allocation;
  pyramid_data_model_agra_CommAllocationPartsType_c comm_allocation;
} pyramid_data_model_agra_ConstrainingPlanPartsType_c;

typedef struct pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c {
  pyramid_slice_t constraining_plan_type;
  uint8_t has_constraining_plan_parts;
  pyramid_data_model_agra_ConstrainingPlanPartsType_c constraining_plan_parts;
} pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c;

typedef struct pyramid_data_model_agra_AutonomousPlanningOtherSystemConstrainingPlansType_c {
  pyramid_slice_t constraining_plan_type;  /* from AutonomousPlanningConstrainingPlansType */
  uint8_t has_constraining_plan_parts;
  pyramid_data_model_agra_ConstrainingPlanPartsType_c constraining_plan_parts;  /* from AutonomousPlanningConstrainingPlansType */
  int32_t constraint_usage;
} pyramid_data_model_agra_AutonomousPlanningOtherSystemConstrainingPlansType_c;

typedef struct pyramid_data_model_agra_MA_PlanPolicyApplicablePlanType_c {
  int32_t plan_type;
  uint8_t has_plan_parts;
  pyramid_data_model_agra_MA_PlanPartsType_c plan_parts;
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c constraining_plans;
  pyramid_slice_t other_system_constraining_plans;
} pyramid_data_model_agra_MA_PlanPolicyApplicablePlanType_c;

typedef struct pyramid_data_model_agra_RequirementFailedTriggerType_c {
  uint8_t has_rank;
  pyramid_data_model_agra_RankCompareType_c rank;  /* from RequirementTriggerType */
  uint8_t has_requirement_types;
  pyramid_data_model_agra_RequirementTaxonomyDetailedType_c requirement_types;  /* from RequirementTriggerType */
  pyramid_slice_t dropped_trigger;
} pyramid_data_model_agra_RequirementFailedTriggerType_c;

typedef struct pyramid_data_model_agra_ZoneViolationTriggerDataType_c {
  pyramid_slice_t category;
} pyramid_data_model_agra_ZoneViolationTriggerDataType_c;

typedef struct pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c {
  double x;
  double y;
  double z;
} pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c;

typedef struct pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c {
  pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c delta_velocity;
  uint8_t has_delta_velocity_magnitude;
  double delta_velocity_magnitude;
  uint8_t has_delta_velocity_covariance;
  pyramid_data_model_agra_CovarianceMatrixType_c delta_velocity_covariance;
  pyramid_str_t duration;
  uint8_t has_delta_mass;
  double delta_mass;
} pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c;

typedef struct pyramid_data_model_agra_SatelliteEnduranceType_c {
  uint8_t has_propulsion;
  pyramid_data_model_agra_EnduranceBaseType_c propulsion;
  uint8_t has_maneuver;
  pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c maneuver;
  uint8_t has_operational;
  pyramid_data_model_agra_EnduranceBaseType_c operational;
} pyramid_data_model_agra_SatelliteEnduranceType_c;

typedef struct pyramid_data_model_agra_CommsLostTriggerDataType_c {
  pyramid_str_t threshold;
} pyramid_data_model_agra_CommsLostTriggerDataType_c;

typedef struct pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c {
  uint8_t has_track_error;
  double track_error;
  uint8_t has_altitude_error;
  double altitude_error;
  uint8_t has_time_error;
  pyramid_data_model_agra_TimeErrorType_c time_error;
} pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c;

typedef struct pyramid_data_model_agra_SystemStateFilterType_c {
  pyramid_slice_t system_state;
} pyramid_data_model_agra_SystemStateFilterType_c;

typedef struct pyramid_data_model_agra_PlanningByCaseTriggerType_c {
  uint8_t has_capability_added;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_added;
  uint8_t has_capability_failure;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_failure;
  uint8_t has_comms_lost;
  pyramid_data_model_agra_CommsLostTriggerDataType_c comms_lost;
  uint8_t has_dmpi_over_designation;
  pyramid_str_t dmpi_over_designation;
  uint8_t has_dmpi_under_designation;
  pyramid_str_t dmpi_under_designation;
  uint8_t has_endurance_low;
  pyramid_data_model_agra_EnduranceType_c endurance_low;
  uint8_t has_off_route;
  pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c off_route;
  uint8_t has_proximity_conflict;
  pyramid_str_t proximity_conflict;
  uint8_t has_release_point_outside_lar;
  pyramid_str_t release_point_outside_lar;
  uint8_t has_route_conflict;
  pyramid_str_t route_conflict;
  uint8_t has_route_vulnerability;
  pyramid_data_model_agra_PlanVulnerabilityType_c route_vulnerability;
  uint8_t has_system_state_change;
  pyramid_data_model_agra_SystemStateFilterType_c system_state_change;
  uint8_t has_requirement_added;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_added;
  uint8_t has_requirement_dependency_failed;
  pyramid_str_t requirement_dependency_failed;
  uint8_t has_requirement_dropped;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_dropped;
  uint8_t has_requirement_failed;
  pyramid_data_model_agra_RequirementFailedTriggerType_c requirement_failed;
  uint8_t has_requirement_change;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_change;
  uint8_t has_requirement_timing;
  pyramid_str_t requirement_timing;
  uint8_t has_zone_violation;
  pyramid_data_model_agra_ZoneViolationTriggerDataType_c zone_violation;
  uint8_t has_orbit_conflict;
  pyramid_str_t orbit_conflict;
  uint8_t has_off_planned_orbit;
  pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c off_planned_orbit;
  uint8_t has_spacecraft_endurance_low;
  pyramid_data_model_agra_SatelliteEnduranceType_c spacecraft_endurance_low;
  uint8_t has_spacecraft_proximity_conflict;
  pyramid_str_t spacecraft_proximity_conflict;
  uint8_t has_response_id;
  pyramid_data_model_agra_ResponseID_Type_c response_id;
} pyramid_data_model_agra_PlanningByCaseTriggerType_c;

typedef struct pyramid_data_model_agra_ByTriggerPolicyType_c {
  int32_t policy;  /* from ApprovalPolicyBaseType */
  uint8_t has_timeout_policy;
  pyramid_data_model_agra_DefaultResponseType_c timeout_policy;  /* from ApprovalPolicyBaseType */
  int32_t trigger_source;
  uint8_t has_trigger;
  pyramid_data_model_agra_PlanningByCaseTriggerType_c trigger;
  pyramid_slice_t by_trigger_by_result_policy;
} pyramid_data_model_agra_ByTriggerPolicyType_c;

typedef struct pyramid_data_model_agra_MA_PlanPolicyType_c {
  pyramid_slice_t applicable_planning_activity;
  uint8_t has_default_policy;
  pyramid_data_model_agra_ApprovalPolicyBaseType_c default_policy;
  pyramid_slice_t by_result_policy;
  pyramid_slice_t by_trigger_policy;
} pyramid_data_model_agra_MA_PlanPolicyType_c;

typedef struct pyramid_data_model_agra_MissionPlanActivationSettingType_c {
  pyramid_slice_t activation_command;
  uint8_t command_subordinate_plans;
} pyramid_data_model_agra_MissionPlanActivationSettingType_c;

typedef struct pyramid_data_model_agra_SubPlanActivationSettingType_c {
  int32_t sub_plan;
  pyramid_slice_t activation_command;
} pyramid_data_model_agra_SubPlanActivationSettingType_c;

typedef struct pyramid_data_model_agra_MA_PlanActivationPolicyType_c {
  pyramid_slice_t by_mission_plan;
  pyramid_slice_t by_sub_plan;
  uint8_t has_by_plan_parts;
  pyramid_data_model_agra_MA_PlanPartsType_c by_plan_parts;
  uint8_t has_policy;
  pyramid_data_model_agra_ApprovalPolicyBaseType_c policy;
} pyramid_data_model_agra_MA_PlanActivationPolicyType_c;

typedef struct pyramid_data_model_agra_ByRequirementPolicyType_c {
  int32_t policy;  /* from ApprovalPolicyBaseType */
  uint8_t has_timeout_policy;
  pyramid_data_model_agra_DefaultResponseType_c timeout_policy;  /* from ApprovalPolicyBaseType */
  pyramid_data_model_agra_RequirementTriggerType_c requirement;
} pyramid_data_model_agra_ByRequirementPolicyType_c;

typedef struct pyramid_data_model_agra_RequirementExecutionPolicyType_c {
  uint8_t has_default_response;
  pyramid_data_model_agra_DefaultResponseType_c default_response;
  pyramid_slice_t by_requirement_policy;
} pyramid_data_model_agra_RequirementExecutionPolicyType_c;

typedef struct pyramid_data_model_agra_MissionTraceabilityType_c {
  int32_t mission_traceability;
  pyramid_str_t traceability_description;
} pyramid_data_model_agra_MissionTraceabilityType_c;

typedef struct pyramid_data_model_agra_ApprovalPolicyID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ApprovalPolicyID_Type_c;

typedef struct pyramid_data_model_agra_MA_ApprovalPolicyMDT_c {
  uint8_t has_data_record_instance_id;
  pyramid_data_model_agra_DataRecordInstanceID_Type_c data_record_instance_id;  /* from DataRecordBaseType */
  pyramid_data_model_agra_ApprovalPolicyID_Type_c approval_policy_id;
  pyramid_slice_t plans;
  pyramid_slice_t plan_activation;
  uint8_t has_requirement_execution;
  pyramid_data_model_agra_RequirementExecutionPolicyType_c requirement_execution;
  uint8_t has_timed_zone;
  pyramid_data_model_agra_TimedZoneType_c timed_zone;
  pyramid_str_t expires;
  uint8_t has_mission_traceability;
  pyramid_data_model_agra_MissionTraceabilityType_c mission_traceability;
} pyramid_data_model_agra_MA_ApprovalPolicyMDT_c;

typedef struct pyramid_data_model_agra_MA_ApprovalPolicyMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MA_ApprovalPolicyMDT_c message_data;
} pyramid_data_model_agra_MA_ApprovalPolicyMT_c;

typedef struct pyramid_data_model_agra_SystemServiceType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_service_id;
  pyramid_data_model_agra_ServiceID_Type_c service_id;
} pyramid_data_model_agra_SystemServiceType_c;

typedef struct pyramid_data_model_agra_OperatorRoleID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OperatorRoleID_Type_c;

typedef struct pyramid_data_model_agra_OperatorRoleType_c {
  uint8_t has_operator_role_id;
  pyramid_data_model_agra_OperatorRoleID_Type_c operator_role_id;
  uint8_t has_non_operator_identifier;
  pyramid_data_model_agra_SystemServiceType_c non_operator_identifier;
} pyramid_data_model_agra_OperatorRoleType_c;

typedef struct pyramid_data_model_agra_RequestID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_RequestID_Type_c;

typedef struct pyramid_data_model_agra_RequestBaseType_c {
  pyramid_data_model_agra_RequestID_Type_c request_id;
  int32_t request_state;
} pyramid_data_model_agra_RequestBaseType_c;

typedef struct pyramid_data_model_agra_DMPI_ID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_DMPI_ID_Type_c;

typedef struct pyramid_data_model_agra_ApprovalRequestItemType_c {
  pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c requirement_id;
  pyramid_slice_t dmpi_id;
} pyramid_data_model_agra_ApprovalRequestItemType_c;

typedef struct pyramid_data_model_agra_ExecutionPlanSetID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ExecutionPlanSetID_Type_c;

typedef struct pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c {
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c execution_plan_set_id;
  int32_t command_type;
} pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c;

typedef struct pyramid_data_model_agra_PathID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_PathID_Type_c;

typedef struct pyramid_data_model_agra_SegmentID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_SegmentID_Type_c;

typedef struct pyramid_data_model_agra_ActivationPathSegmentType_c {
  pyramid_data_model_agra_PathID_Type_c path_id;
  pyramid_data_model_agra_SegmentID_Type_c segment_id;
} pyramid_data_model_agra_ActivationPathSegmentType_c;

typedef struct pyramid_data_model_agra_RouteActivityPlanActivationType_c {
  pyramid_data_model_agra_RouteActivityPlanID_Type_c route_activity_plan_id;
  int32_t command_type;
  uint8_t has_activation_activity_id;
  pyramid_data_model_agra_PlannedActivityID_Type_c activation_activity_id;
  uint8_t has_activation_path_segment;
  pyramid_data_model_agra_ActivationPathSegmentType_c activation_path_segment;
} pyramid_data_model_agra_RouteActivityPlanActivationType_c;

typedef struct pyramid_data_model_agra_RoutePlanActivationType_c {
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  int32_t command_type;
  uint8_t has_activation_path_segment;
  pyramid_data_model_agra_ActivationPathSegmentType_c activation_path_segment;
} pyramid_data_model_agra_RoutePlanActivationType_c;

typedef struct pyramid_data_model_agra_ActivityPlanActivationType_c {
  pyramid_data_model_agra_ActivityPlanID_Type_c activity_plan_id;
  int32_t command_type;
  uint8_t has_activation_activity_id;
  pyramid_data_model_agra_PlannedActivityID_Type_c activation_activity_id;
} pyramid_data_model_agra_ActivityPlanActivationType_c;

typedef struct pyramid_data_model_agra_EffectPlanActivationType_c {
  pyramid_data_model_agra_EffectPlanID_Type_c effect_plan_id;
  int32_t command_type;
} pyramid_data_model_agra_EffectPlanActivationType_c;

typedef struct pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from VersionedID_Type */
  uint8_t has_version;
  uint32_t version;  /* from VersionedID_Type */
} pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c;

typedef struct pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c;

typedef struct pyramid_data_model_agra_ActivationOrbitSequenceType_c {
  pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c sequence_id;
  uint8_t has_segment_id;
  pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c segment_id;
} pyramid_data_model_agra_ActivationOrbitSequenceType_c;

typedef struct pyramid_data_model_agra_OrbitPlanActivationType_c {
  pyramid_data_model_agra_OrbitPlanID_Type_c orbit_plan_id;
  int32_t command_type;
  uint8_t has_activation_orbit_sequence;
  pyramid_data_model_agra_ActivationOrbitSequenceType_c activation_orbit_sequence;
} pyramid_data_model_agra_OrbitPlanActivationType_c;

typedef struct pyramid_data_model_agra_OrbitActivityPlanActivationType_c {
  pyramid_data_model_agra_OrbitActivityPlanID_Type_c orbit_activity_plan_id;
  int32_t command_type;
  uint8_t has_activation_activity_id;
  pyramid_data_model_agra_PlannedActivityID_Type_c activation_activity_id;
  uint8_t has_activation_orbit_sequence;
  pyramid_data_model_agra_ActivationOrbitSequenceType_c activation_orbit_sequence;
} pyramid_data_model_agra_OrbitActivityPlanActivationType_c;

typedef struct pyramid_data_model_agra_ActionPlanActivationType_c {
  pyramid_data_model_agra_ActionPlanID_Type_c action_plan_id;
  int32_t command_type;
} pyramid_data_model_agra_ActionPlanActivationType_c;

typedef struct pyramid_data_model_agra_TaskPlanActivationType_c {
  pyramid_data_model_agra_TaskPlanID_Type_c task_plan_id;
  int32_t command_type;
} pyramid_data_model_agra_TaskPlanActivationType_c;

typedef struct pyramid_data_model_agra_MA_OpPlanActivationType_c {
  pyramid_data_model_agra_MA_OpPlanID_Type_c op_plan_id;
  int32_t command_type;
} pyramid_data_model_agra_MA_OpPlanActivationType_c;

typedef struct pyramid_data_model_agra_ResponsePlanActivationType_c {
  pyramid_data_model_agra_ResponsePlanID_Type_c response_plan_id;
  int32_t command_type;
} pyramid_data_model_agra_ResponsePlanActivationType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c {
  pyramid_slice_t task_plan;
  pyramid_slice_t route_plan;
  pyramid_slice_t route_activity_plan;
  pyramid_slice_t orbit_plan;
  pyramid_slice_t orbit_activity_plan;
  pyramid_slice_t activity_plan;
  pyramid_slice_t effect_plan;
  pyramid_slice_t action_plan;
  pyramid_slice_t response_plan;
  pyramid_slice_t op_plan;
} pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c;

typedef struct pyramid_data_model_agra_MissionPlanActivationType_c {
  int32_t activation_command;
  uint8_t command_subordinate_plans;
  pyramid_slice_t subordinate_mission_plan_id;
} pyramid_data_model_agra_MissionPlanActivationType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c {
  uint8_t has_by_mission_plan;
  pyramid_data_model_agra_MissionPlanActivationType_c by_mission_plan;
  uint8_t has_by_sub_plan;
  pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c by_sub_plan;
  uint8_t has_by_execution_plan_set;
  pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c by_execution_plan_set;
} pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c {
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c activation_details;
} pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c;

typedef struct pyramid_data_model_agra_CommScheduleAllocationID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_CommScheduleAllocationID_Type_c;

typedef struct pyramid_data_model_agra_PlanReferenceID_ChoiceType_c {
  uint8_t has_mission_plan_id;
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  uint8_t has_task_plan_id;
  pyramid_data_model_agra_TaskPlanID_Type_c task_plan_id;
  uint8_t has_orbit_plan_id;
  pyramid_data_model_agra_OrbitPlanID_Type_c orbit_plan_id;
  uint8_t has_orbit_activity_plan_id;
  pyramid_data_model_agra_OrbitActivityPlanID_Type_c orbit_activity_plan_id;
  uint8_t has_route_plan_id;
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  uint8_t has_route_activity_plan_id;
  pyramid_data_model_agra_RouteActivityPlanID_Type_c route_activity_plan_id;
  uint8_t has_comm_schedule_allocation_id;
  pyramid_data_model_agra_CommScheduleAllocationID_Type_c comm_schedule_allocation_id;
  uint8_t has_activity_plan_id;
  pyramid_data_model_agra_ActivityPlanID_Type_c activity_plan_id;
  uint8_t has_effect_plan_id;
  pyramid_data_model_agra_EffectPlanID_Type_c effect_plan_id;
  uint8_t has_action_plan_id;
  pyramid_data_model_agra_ActionPlanID_Type_c action_plan_id;
  uint8_t has_response_plan_id;
  pyramid_data_model_agra_ResponsePlanID_Type_c response_plan_id;
} pyramid_data_model_agra_PlanReferenceID_ChoiceType_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c {
  uint8_t has_plan_approval;
  pyramid_data_model_agra_PlanReferenceID_ChoiceType_c plan_approval;
  uint8_t has_requirement_execution_approval;
  pyramid_data_model_agra_ApprovalRequestItemType_c requirement_execution_approval;
  uint8_t has_mission_plan_activation_approval;
  pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c mission_plan_activation_approval;
} pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c {
  pyramid_data_model_agra_ApprovalPolicyID_Type_c approval_policy_id;
  pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c approval_item;
} pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestMDT_c {
  pyramid_data_model_agra_RequestID_Type_c request_id;  /* from RequestBaseType */
  int32_t request_state;  /* from RequestBaseType */
  pyramid_data_model_agra_OperatorRoleType_c approver;
  pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c approval_references;
  pyramid_str_t respond_by;
} pyramid_data_model_agra_MA_ApprovalRequestMDT_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_ApprovalRequestMDT_c message_data;
} pyramid_data_model_agra_MA_ApprovalRequestMT_c;

typedef struct pyramid_data_model_agra_OperatorID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_OperatorID_Type_c;

typedef struct pyramid_data_model_agra_RequestStatusBaseType_c {
  pyramid_data_model_agra_RequestID_Type_c request_id;
  int32_t request_processing_state;
  uint8_t has_request_processing_state_reason;
  pyramid_data_model_agra_CannotComplyType_c request_processing_state_reason;
} pyramid_data_model_agra_RequestStatusBaseType_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c {
  pyramid_data_model_agra_RequestID_Type_c request_id;  /* from RequestStatusBaseType */
  int32_t request_processing_state;  /* from RequestStatusBaseType */
  uint8_t has_request_processing_state_reason;
  pyramid_data_model_agra_CannotComplyType_c request_processing_state_reason;  /* from RequestStatusBaseType */
  int32_t approval_request_processing_state;
  uint8_t has_approval_request_processing_state_reason;
  pyramid_data_model_agra_CannotComplyType_c approval_request_processing_state_reason;
  uint8_t has_operator_id;
  pyramid_data_model_agra_OperatorID_Type_c operator_id;
  pyramid_slice_t approved_dmpi_id;
} pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c message_data;
} pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c;

typedef struct pyramid_data_model_agra_MissionPlanCommandID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MissionPlanCommandID_Type_c;

typedef struct pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c {
  uint8_t has_mission_plan_command_id;
  pyramid_data_model_agra_MissionPlanCommandID_Type_c mission_plan_command_id;
  uint8_t has_mission_plan_validation_command_id;
  pyramid_data_model_agra_CommandID_Type_c mission_plan_validation_command_id;
} pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c;

typedef struct pyramid_data_model_agra_MA_PlanApplicabilityType_c {
  uint8_t has_planned_for_id;
  pyramid_data_model_agra_SystemID_Type_c planned_for_id;
  pyramid_data_model_agra_MA_PackageSystemType_c applicable_to_i_ds;
} pyramid_data_model_agra_MA_PlanApplicabilityType_c;

typedef struct pyramid_data_model_agra_ExecutionPlanSetBaseType_c {
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c execution_plan_set_id;
  uint8_t has_next_execution_plan_set_id;
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c next_execution_plan_set_id;
  pyramid_slice_t activity_plan_id;
  pyramid_slice_t comm_schedule_allocation_id;
} pyramid_data_model_agra_ExecutionPlanSetBaseType_c;

typedef struct pyramid_data_model_agra_RouteExecutionPlanSetType_c {
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c execution_plan_set_id;  /* from ExecutionPlanSetBaseType */
  uint8_t has_next_execution_plan_set_id;
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c next_execution_plan_set_id;  /* from ExecutionPlanSetBaseType */
  pyramid_slice_t activity_plan_id;  /* from ExecutionPlanSetBaseType */
  pyramid_slice_t comm_schedule_allocation_id;  /* from ExecutionPlanSetBaseType */
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  pyramid_slice_t route_activity_plan_id;
} pyramid_data_model_agra_RouteExecutionPlanSetType_c;

typedef struct pyramid_data_model_agra_OrbitExecutionPlanSetType_c {
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c execution_plan_set_id;  /* from ExecutionPlanSetBaseType */
  uint8_t has_next_execution_plan_set_id;
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c next_execution_plan_set_id;  /* from ExecutionPlanSetBaseType */
  pyramid_slice_t activity_plan_id;  /* from ExecutionPlanSetBaseType */
  pyramid_slice_t comm_schedule_allocation_id;  /* from ExecutionPlanSetBaseType */
  pyramid_data_model_agra_OrbitPlanID_Type_c orbit_plan_id;
  pyramid_slice_t orbit_activity_plan_id;
} pyramid_data_model_agra_OrbitExecutionPlanSetType_c;

typedef struct pyramid_data_model_agra_ExecutionSequencePlanSetsType_c {
  pyramid_slice_t route_execution_plan_set;
  pyramid_slice_t orbit_execution_plan_set;
  pyramid_slice_t activity_execution_plan_set;
} pyramid_data_model_agra_ExecutionSequencePlanSetsType_c;

typedef struct pyramid_data_model_agra_ExecutionSequenceType_c {
  pyramid_slice_t route_execution_plan_set;  /* from ExecutionSequencePlanSetsType */
  pyramid_slice_t orbit_execution_plan_set;  /* from ExecutionSequencePlanSetsType */
  pyramid_slice_t activity_execution_plan_set;  /* from ExecutionSequencePlanSetsType */
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c initial_execution_plan_set_id;
} pyramid_data_model_agra_ExecutionSequenceType_c;

typedef struct pyramid_data_model_agra_ActionPlanConstraintType_c {
  uint8_t has_action_plan_id;
  pyramid_data_model_agra_ActionPlanID_Type_c action_plan_id;
  int32_t changeable_allocations;
  pyramid_slice_t action_type;
} pyramid_data_model_agra_ActionPlanConstraintType_c;

typedef struct pyramid_data_model_agra_ResponsePlanConstraintType_c {
  uint8_t has_response_plan_id;
  pyramid_data_model_agra_ResponsePlanID_Type_c response_plan_id;
  int32_t changeable_allocations;
  pyramid_slice_t response_type;
} pyramid_data_model_agra_ResponsePlanConstraintType_c;

typedef struct pyramid_data_model_agra_EffectPlanConstraintType_c {
  uint8_t has_effect_plan_id;
  pyramid_data_model_agra_EffectPlanID_Type_c effect_plan_id;
  int32_t changeable_allocations;
  pyramid_slice_t effect_type;
} pyramid_data_model_agra_EffectPlanConstraintType_c;

typedef struct pyramid_data_model_agra_MA_TaskPlanConstraintType_c {
  uint8_t has_allocation_plan_id;
  pyramid_data_model_agra_TaskPlanID_Type_c allocation_plan_id;
  int32_t changeable_allocations;
  pyramid_slice_t task_type;
} pyramid_data_model_agra_MA_TaskPlanConstraintType_c;

typedef struct pyramid_data_model_agra_MA_RequirementPlanConstraintType_c {
  uint8_t has_proposed_effect_plan;
  pyramid_data_model_agra_EffectPlanConstraintType_c proposed_effect_plan;
  uint8_t has_proposed_action_plan;
  pyramid_data_model_agra_ActionPlanConstraintType_c proposed_action_plan;
  uint8_t has_proposed_task_plan;
  pyramid_data_model_agra_MA_TaskPlanConstraintType_c proposed_task_plan;
  uint8_t has_proposed_response_plan;
  pyramid_data_model_agra_ResponsePlanConstraintType_c proposed_response_plan;
} pyramid_data_model_agra_MA_RequirementPlanConstraintType_c;

typedef struct pyramid_data_model_agra_OrbitTransitionSequenceType_c {
  pyramid_data_model_agra_OrbitPlanID_Type_c plan_id;
  pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c sequence_id;
  uint8_t has_segment_id;
  pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c segment_id;
} pyramid_data_model_agra_OrbitTransitionSequenceType_c;

typedef struct pyramid_data_model_agra_OrbitPlanningStateType_c {
  pyramid_str_t time;
  uint8_t has_kinematics;
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c kinematics;
  uint8_t has_element_set;
  pyramid_data_model_agra_TLE_BaseType_c element_set;
  uint8_t has_endurance;
  pyramid_data_model_agra_SatelliteEnduranceType_c endurance;
  uint8_t has_transition;
  pyramid_data_model_agra_OrbitTransitionSequenceType_c transition;
} pyramid_data_model_agra_OrbitPlanningStateType_c;

typedef struct pyramid_data_model_agra_OrbitGuidelineType_c {
  uint8_t has_destination;
  pyramid_data_model_agra_OrbitPlanningStateType_c destination;
  uint8_t has_origin;
  pyramid_data_model_agra_OrbitPlanningStateType_c origin;
} pyramid_data_model_agra_OrbitGuidelineType_c;

typedef struct pyramid_data_model_agra_RouteActivityPlanConstraintType_c {
  uint8_t has_route_activity_plan_id;
  pyramid_data_model_agra_RouteActivityPlanID_Type_c route_activity_plan_id;
  int32_t changeable_activities;
  pyramid_slice_t parts;
} pyramid_data_model_agra_RouteActivityPlanConstraintType_c;

typedef struct pyramid_data_model_agra_ActivityPlanConstraintType_c {
  uint8_t has_activity_plan_id;
  pyramid_data_model_agra_ActivityPlanID_Type_c activity_plan_id;
  int32_t changeable_activities;
  pyramid_slice_t parts;
} pyramid_data_model_agra_ActivityPlanConstraintType_c;

typedef struct pyramid_data_model_agra_OrbitActivityPlanConstraintType_c {
  uint8_t has_orbit_activity_plan_id;
  pyramid_data_model_agra_OrbitActivityPlanID_Type_c orbit_activity_plan_id;
  int32_t changeable_activities;
  pyramid_slice_t parts;
} pyramid_data_model_agra_OrbitActivityPlanConstraintType_c;

typedef struct pyramid_data_model_agra_OrbitPlanConstraintType_c {
  uint8_t has_orbit_plan_id;
  pyramid_data_model_agra_OrbitPlanID_Type_c orbit_plan_id;
  int32_t changeable_kinematics;
  pyramid_slice_t parts;
} pyramid_data_model_agra_OrbitPlanConstraintType_c;

typedef struct pyramid_data_model_agra_CommAllocationConstraintType_c {
  uint8_t has_allocation_id;
  pyramid_data_model_agra_CommScheduleAllocationID_Type_c allocation_id;
  int32_t changeable_allocation;
  pyramid_slice_t parts;
} pyramid_data_model_agra_CommAllocationConstraintType_c;

typedef struct pyramid_data_model_agra_RoutePlanConstraintType_c {
  uint8_t has_route_plan_id;
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  int32_t changeable_kinematics;
  pyramid_slice_t parts;
} pyramid_data_model_agra_RoutePlanConstraintType_c;

typedef struct pyramid_data_model_agra_MA_ConstrainingPlansType_c {
  pyramid_slice_t constraining_plan_type;
  pyramid_slice_t mission_plan_id;
  pyramid_slice_t task_plan;
  pyramid_slice_t route_plan;
  pyramid_slice_t route_activity_plan;
  pyramid_slice_t orbit_plan;
  pyramid_slice_t orbit_activity_plan;
  pyramid_slice_t activity_plan;
  pyramid_slice_t comm_allocation;
  pyramid_slice_t effect_plan;
  pyramid_slice_t action_plan;
  pyramid_slice_t response_plan;
} pyramid_data_model_agra_MA_ConstrainingPlansType_c;

typedef struct pyramid_data_model_agra_MA_OtherSystemConstrainingPlansType_c {
  pyramid_data_model_agra_SystemID_Type_c constraining_system_id;
  int32_t constraint_usage;
  pyramid_data_model_agra_MA_ConstrainingPlansType_c contraining_plans;
} pyramid_data_model_agra_MA_OtherSystemConstrainingPlansType_c;

typedef struct pyramid_data_model_agra_MA_PlanningCandidateBaseType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_MA_ConstrainingPlansType_c constraining_plans;
  pyramid_slice_t other_system_constraining_plans;
} pyramid_data_model_agra_MA_PlanningCandidateBaseType_c;

typedef struct pyramid_data_model_agra_RoutePlanReferenceType_c {
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  uint8_t has_mission_plan_id;
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  uint8_t has_path_id;
  pyramid_data_model_agra_PathID_Type_c path_id;
  uint8_t has_segment_id;
  pyramid_data_model_agra_SegmentID_Type_c segment_id;
} pyramid_data_model_agra_RoutePlanReferenceType_c;

typedef struct pyramid_data_model_agra_Point4D_Type_c {
  double latitude;
  double longitude;
  double altitude;
  uint8_t has_altitude_reference;
  int32_t altitude_reference;
  pyramid_str_t timestamp;
  uint8_t has_depth_category;
  int32_t depth_category;
  uint8_t has_hae_adjustment;
  double hae_adjustment;
} pyramid_data_model_agra_Point4D_Type_c;

typedef struct pyramid_data_model_agra_RelativeOffset3D_Type_c {
  int32_t rotation;
  int32_t xy_offsets;
  double x;
  double y;
  pyramid_data_model_agra_Z_ChoiceType_c z;
} pyramid_data_model_agra_RelativeOffset3D_Type_c;

typedef struct pyramid_data_model_agra_Point4D_RelativeType_c {
  pyramid_data_model_agra_ReferenceFrameID_Type_c reference_frame_id;
  pyramid_str_t timestamp;
  uint8_t has_relative_offset;
  pyramid_data_model_agra_RelativeOffset3D_Type_c relative_offset;
} pyramid_data_model_agra_Point4D_RelativeType_c;

typedef struct pyramid_data_model_agra_PointChoice4D_Type_c {
  uint8_t has_absolute_point;
  pyramid_data_model_agra_Point4D_Type_c absolute_point;
  uint8_t has_relative_point;
  pyramid_data_model_agra_Point4D_RelativeType_c relative_point;
} pyramid_data_model_agra_PointChoice4D_Type_c;

typedef struct pyramid_data_model_agra_Velocity3D_Type_c {
  double north_speed;
  double east_speed;
  double down_speed;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_Velocity3D_Type_c;

typedef struct pyramid_data_model_agra_OrientationRateType_c {
  double yaw_rate;
  double pitch_rate;
  double roll_rate;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_OrientationRateType_c;

typedef struct pyramid_data_model_agra_InertialStateRelativeType_c {
  pyramid_data_model_agra_PointChoice4D_Type_c position;
  uint8_t has_position_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c position_uncertainty;
  uint8_t has_domain_velocity;
  pyramid_data_model_agra_Velocity3D_Type_c domain_velocity;
  uint8_t has_ground_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c ground_velocity;
  uint8_t has_relative_velocity;
  pyramid_data_model_agra_Velocity3D_Type_c relative_velocity;
  uint8_t has_domain_acceleration;
  pyramid_data_model_agra_Acceleration3D_Type_c domain_acceleration;
  uint8_t has_orientation;
  pyramid_data_model_agra_OrientationType_c orientation;
  uint8_t has_orientation_rate;
  pyramid_data_model_agra_OrientationRateType_c orientation_rate;
} pyramid_data_model_agra_InertialStateRelativeType_c;

typedef struct pyramid_data_model_agra_VehicleConfigurationType_c {
  pyramid_data_model_agra_InertialStateRelativeType_c inertial_state;
  uint8_t has_endurance;
  pyramid_data_model_agra_EnduranceType_c endurance;
  uint8_t has_gross_weight;
  double gross_weight;
} pyramid_data_model_agra_VehicleConfigurationType_c;

typedef struct pyramid_data_model_agra_PlanningLocationType_c {
  pyramid_data_model_agra_InertialStateRelativeType_c inertial_state;  /* from VehicleConfigurationType */
  uint8_t has_endurance;
  pyramid_data_model_agra_EnduranceType_c endurance;  /* from VehicleConfigurationType */
  uint8_t has_gross_weight;
  double gross_weight;  /* from VehicleConfigurationType */
  uint8_t has_transition_route;
  pyramid_data_model_agra_RoutePlanReferenceType_c transition_route;
} pyramid_data_model_agra_PlanningLocationType_c;

typedef struct pyramid_data_model_agra_PlanningPointType_c {
  uint8_t has_specific;
  pyramid_data_model_agra_PlanningLocationType_c specific;
  uint8_t has_op_point_id;
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
  uint8_t has_time;
  pyramid_str_t time;
} pyramid_data_model_agra_PlanningPointType_c;

typedef struct pyramid_data_model_agra_PlanningPointPriorityType_c {
  pyramid_data_model_agra_PlanningPointType_c point;
  uint8_t has_priority;
  uint32_t priority;
} pyramid_data_model_agra_PlanningPointPriorityType_c;

typedef struct pyramid_data_model_agra_PlanningGuidelineType_c {
  uint8_t has_origin;
  pyramid_data_model_agra_PlanningPointType_c origin;
  pyramid_slice_t destination;
  uint8_t has_speed_optimization;
  int32_t speed_optimization;
  uint8_t has_climb_optimization;
  int32_t climb_optimization;
} pyramid_data_model_agra_PlanningGuidelineType_c;

typedef struct pyramid_data_model_agra_MA_RequirementPlanningCandidateType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;  /* from MA_PlanningCandidateBaseType */
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_MA_ConstrainingPlansType_c constraining_plans;  /* from MA_PlanningCandidateBaseType */
  pyramid_slice_t other_system_constraining_plans;  /* from MA_PlanningCandidateBaseType */
  uint8_t has_route_guideline;
  pyramid_data_model_agra_PlanningGuidelineType_c route_guideline;
  uint8_t has_orbit_guideline;
  pyramid_data_model_agra_OrbitGuidelineType_c orbit_guideline;
} pyramid_data_model_agra_MA_RequirementPlanningCandidateType_c;

typedef struct pyramid_data_model_agra_ConstrainedOpLineType_c {
  int32_t category;
  pyramid_data_model_agra_OpLineID_Type_c op_line_id;
} pyramid_data_model_agra_ConstrainedOpLineType_c;

typedef struct pyramid_data_model_agra_ConstrainedEntityType_c {
  uint8_t has_surrogate_identifier;
  pyramid_data_model_agra_RadarEmitterIdentityType_c surrogate_identifier;
  uint8_t has_mission_planning_weighting_factor;
  double mission_planning_weighting_factor;
  uint8_t has_mission_planning_candidate;
  uint8_t mission_planning_candidate;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_ConstrainedEntityType_c;

typedef struct pyramid_data_model_agra_Point3D_Type_c {
  double latitude;
  double longitude;
  double altitude;
  uint8_t has_altitude_reference;
  int32_t altitude_reference;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_Point3D_Type_c;

typedef struct pyramid_data_model_agra_IngressEgressType_c {
  uint8_t has_entry_point;
  pyramid_data_model_agra_Point3D_Type_c entry_point;
  uint8_t has_exit_point;
  pyramid_data_model_agra_Point3D_Type_c exit_point;
} pyramid_data_model_agra_IngressEgressType_c;

typedef struct pyramid_data_model_agra_CommAllocationActionType_c {
  pyramid_data_model_agra_CommScheduleAllocationID_Type_c comm_allocation_id;
  int32_t comm_allocation_transition;
} pyramid_data_model_agra_CommAllocationActionType_c;

typedef struct pyramid_data_model_agra_VehicleCommandDataType_c {
  uint8_t has_vehicle_action;
  int32_t vehicle_action;
  uint8_t has_iff_settings;
  pyramid_data_model_agra_IFF_Type_c iff_settings;
  uint8_t has_survivability_mode;
  int32_t survivability_mode;
  pyramid_str_t lost_comm_timeout;
  uint8_t has_los;
  uint8_t los;
  uint8_t has_loss_of_link_processing;
  int32_t loss_of_link_processing;
  uint8_t has_comm_allocation_action;
  pyramid_data_model_agra_CommAllocationActionType_c comm_allocation_action;
} pyramid_data_model_agra_VehicleCommandDataType_c;

typedef struct pyramid_data_model_agra_NetworkLinkID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_NetworkLinkID_Type_c;

typedef struct pyramid_data_model_agra_Link16TrackIdentifierType_c {
  pyramid_str_t track_number;
  uint8_t has_external_interface_id;
  pyramid_data_model_agra_NetworkLinkID_Type_c external_interface_id;
} pyramid_data_model_agra_Link16TrackIdentifierType_c;

typedef struct pyramid_data_model_agra_TrackNumberOrEntityType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_track_number;
  pyramid_data_model_agra_Link16TrackIdentifierType_c track_number;
} pyramid_data_model_agra_TrackNumberOrEntityType_c;

typedef struct pyramid_data_model_agra_OpZoneMissileDataType_c {
  uint8_t has_space_object;
  int32_t space_object;
  uint8_t has_confidence;
  pyramid_data_model_agra_PercentRangeType_c confidence;
  uint8_t has_related_track;
  pyramid_data_model_agra_TrackNumberOrEntityType_c related_track;
  uint8_t has_launch_point_calculation;
  int32_t launch_point_calculation;
} pyramid_data_model_agra_OpZoneMissileDataType_c;

typedef struct pyramid_data_model_agra_OpZoneNoFlyType_c {
  double percent_restricted;
} pyramid_data_model_agra_OpZoneNoFlyType_c;

typedef struct pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c;

typedef struct pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c;

typedef struct pyramid_data_model_agra_WeaponRestrictionType_c {
  uint8_t has_weapons_allowed;
  pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c weapons_allowed;
  uint8_t has_weapons_not_allowed;
  pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c weapons_not_allowed;
} pyramid_data_model_agra_WeaponRestrictionType_c;

typedef struct pyramid_data_model_agra_OpZoneWeaponRestrictionType_c {
  pyramid_data_model_agra_WeaponRestrictionType_c weapon;
  uint8_t has_identity;
  pyramid_data_model_agra_IdentityType_c identity;
} pyramid_data_model_agra_OpZoneWeaponRestrictionType_c;

typedef struct pyramid_data_model_agra_FrequencyRangeType_c {
  double min;
  double max;
} pyramid_data_model_agra_FrequencyRangeType_c;

typedef struct pyramid_data_model_agra_OpZoneJammingType_c {
  uint8_t allowed;
  pyramid_slice_t restricted_frequency_band;
} pyramid_data_model_agra_OpZoneJammingType_c;

typedef struct pyramid_data_model_agra_OpZoneFilterAreaPET_c {
  int32_t filter_type;
  uint8_t has_ranking;
  pyramid_data_model_agra_ComparableRankingType_c ranking;
} pyramid_data_model_agra_OpZoneFilterAreaPET_c;

typedef struct pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c;

typedef struct pyramid_data_model_agra_ConstrainedEntryExitType_c {
  pyramid_slice_t gate_index;
} pyramid_data_model_agra_ConstrainedEntryExitType_c;

typedef struct pyramid_data_model_agra_WindMagnitudeType_c {
  uint8_t has_wind_direction;
  double wind_direction;
  uint8_t has_wind_speed;
  double wind_speed;
} pyramid_data_model_agra_WindMagnitudeType_c;

typedef struct pyramid_data_model_agra_WindDataChoiceType_c {
  uint8_t has_wind_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c wind_velocity;
  uint8_t has_wind_magnitude;
  pyramid_data_model_agra_WindMagnitudeType_c wind_magnitude;
} pyramid_data_model_agra_WindDataChoiceType_c;

typedef struct pyramid_data_model_agra_WindDataType_c {
  uint8_t has_wind_choice;
  pyramid_data_model_agra_WindDataChoiceType_c wind_choice;
  uint8_t has_wind_gust;
  double wind_gust;
  pyramid_str_t wind_shear;
  pyramid_str_t variable_winds;
} pyramid_data_model_agra_WindDataType_c;

typedef struct pyramid_data_model_agra_CloudsType_c {
  uint8_t has_cloud_cover;
  int32_t cloud_cover;
  uint8_t has_cloud_cover_amplification;
  uint32_t cloud_cover_amplification;
  uint8_t has_cloud_floor;
  double cloud_floor;
  uint8_t has_cloud_ceiling;
  double cloud_ceiling;
} pyramid_data_model_agra_CloudsType_c;

typedef struct pyramid_data_model_agra_WeatherEffectsType_c {
  uint8_t has_road_state;
  int32_t road_state;
  uint8_t has_terrain_state;
  int32_t terrain_state;
  uint8_t has_sea_state;
  int32_t sea_state;
  uint8_t has_sea_state_amplification;
  int32_t sea_state_amplification;
  uint8_t has_system_icing_state;
  uint8_t system_icing_state;
} pyramid_data_model_agra_WeatherEffectsType_c;

typedef struct pyramid_data_model_agra_WeatherAreaDataType_c {
  int32_t source;
  pyramid_str_t station_code;
  uint8_t has_temperature;
  double temperature;
  uint8_t has_precipitation_potential;
  double precipitation_potential;
  uint8_t has_precipitation_amount;
  double precipitation_amount;
  uint8_t has_type_of_precipitation;
  int32_t type_of_precipitation;
  uint8_t has_precipitation_amplification;
  int32_t precipitation_amplification;
  uint8_t has_visibility;
  double visibility;
  uint8_t has_visibility_status;
  int32_t visibility_status;
  uint8_t has_dew_point;
  double dew_point;
  pyramid_slice_t clouds;
  uint8_t has_weather_effects;
  pyramid_data_model_agra_WeatherEffectsType_c weather_effects;
  uint8_t has_barometric_pressure;
  float barometric_pressure;
  uint8_t has_kollsman_setting;
  float kollsman_setting;
  uint8_t has_icing;
  int32_t icing;
  uint8_t has_turbulence;
  int32_t turbulence;
  uint8_t has_thunderstorm_potential;
  double thunderstorm_potential;
  pyramid_str_t remarks;
  uint8_t has_kind;
  int32_t kind;
  uint8_t has_description;
  int32_t description;
  uint8_t has_qualifier;
  int32_t qualifier;
  uint8_t has_wind_data;
  pyramid_data_model_agra_WindDataType_c wind_data;
  uint8_t has_relative_humidity;
  double relative_humidity;
} pyramid_data_model_agra_WeatherAreaDataType_c;

typedef struct pyramid_data_model_agra_OpZoneWeatherType_c {
  pyramid_slice_t weather_warning_type;
  uint8_t has_area_data;
  pyramid_data_model_agra_WeatherAreaDataType_c area_data;
} pyramid_data_model_agra_OpZoneWeatherType_c;

typedef struct pyramid_data_model_agra_OpZoneNoFireType_c {
  pyramid_slice_t entity_class;
} pyramid_data_model_agra_OpZoneNoFireType_c;

typedef struct pyramid_data_model_agra_OpZoneCategoryType_c {
  uint8_t has_constrained_entry_exit;
  pyramid_data_model_agra_ConstrainedEntryExitType_c constrained_entry_exit;
  uint8_t has_filter_area;
  pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c filter_area;
  uint8_t has_jamming;
  pyramid_data_model_agra_OpZoneJammingType_c jamming;
  uint8_t has_keep_in;
  pyramid_data_model_agra_IngressEgressType_c keep_in;
  uint8_t has_missile_launch_point;
  pyramid_data_model_agra_OpZoneMissileDataType_c missile_launch_point;
  uint8_t has_no_fire;
  pyramid_data_model_agra_OpZoneNoFireType_c no_fire;
  uint8_t has_no_fly;
  pyramid_data_model_agra_OpZoneNoFlyType_c no_fly;
  uint8_t has_vehicle_configuration;
  pyramid_data_model_agra_VehicleCommandDataType_c vehicle_configuration;
  uint8_t has_weapon_restriction;
  pyramid_data_model_agra_OpZoneWeaponRestrictionType_c weapon_restriction;
  uint8_t has_weather_conditions;
  pyramid_data_model_agra_OpZoneWeatherType_c weather_conditions;
} pyramid_data_model_agra_OpZoneCategoryType_c;

typedef struct pyramid_data_model_agra_ConstrainedOpVolumeType_c {
  int32_t volume_category;
  uint8_t has_category_unique_data;
  pyramid_data_model_agra_OpZoneCategoryType_c category_unique_data;
  pyramid_data_model_agra_OpVolumeID_Type_c op_volume_id;
} pyramid_data_model_agra_ConstrainedOpVolumeType_c;

typedef struct pyramid_data_model_agra_ConstrainedOpZoneType_c {
  int32_t zone_category;
  uint8_t has_category_unique_data;
  pyramid_data_model_agra_OpZoneCategoryType_c category_unique_data;
  pyramid_data_model_agra_OpZoneID_Type_c op_zone_id;
} pyramid_data_model_agra_ConstrainedOpZoneType_c;

typedef struct pyramid_data_model_agra_SignalReportID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SignalReportID_Type_c;

typedef struct pyramid_data_model_agra_GeoLocatedObjectType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_op_point_id;
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
  uint8_t has_op_line_id;
  pyramid_data_model_agra_OpLineID_Type_c op_line_id;
  uint8_t has_op_zone_id;
  pyramid_data_model_agra_OpZoneID_Type_c op_zone_id;
  uint8_t has_op_volume_id;
  pyramid_data_model_agra_OpVolumeID_Type_c op_volume_id;
  uint8_t has_dmpi_id;
  pyramid_data_model_agra_DMPI_ID_Type_c dmpi_id;
  uint8_t has_signal_report_id;
  pyramid_data_model_agra_SignalReportID_Type_c signal_report_id;
} pyramid_data_model_agra_GeoLocatedObjectType_c;

typedef struct pyramid_data_model_agra_OpZoneType_c {
  pyramid_data_model_agra_AreaChoiceType_c shape;  /* from ZoneType */
  uint8_t has_min_altitude;
  double min_altitude;  /* from ZoneType */
  uint8_t has_max_altitude;
  double max_altitude;  /* from ZoneType */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from ZoneType */
  uint8_t has_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c velocity;
} pyramid_data_model_agra_OpZoneType_c;

typedef struct pyramid_data_model_agra_SystemMessageIdentifierType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_service_id;
  pyramid_data_model_agra_ServiceID_Type_c service_id;
  uint8_t has_activity_id;
  pyramid_data_model_agra_ActivityID_Type_c activity_id;
  uint8_t has_network_link_id;
  pyramid_data_model_agra_NetworkLinkID_Type_c network_link_id;
} pyramid_data_model_agra_SystemMessageIdentifierType_c;

typedef struct pyramid_data_model_agra_QualifyingTagsType_c {
  uint8_t has_qualified_special;
  uint32_t qualified_special;
  pyramid_str_t process_special;
  pyramid_str_t filter_override;
  pyramid_str_t high_interest;
  pyramid_str_t retain;
  pyramid_data_model_agra_SystemMessageIdentifierType_c qualifying_source;
} pyramid_data_model_agra_QualifyingTagsType_c;

typedef struct pyramid_data_model_agra_OpDescriptionType_c {
  pyramid_str_t name;
  pyramid_str_t remarks;
} pyramid_data_model_agra_OpDescriptionType_c;

typedef struct pyramid_data_model_agra_TimeFunctionType_c {
  int32_t function;
  pyramid_str_t value;
} pyramid_data_model_agra_TimeFunctionType_c;

typedef struct pyramid_data_model_agra_ScheduleStateType_c {
  int32_t state;
  pyramid_data_model_agra_ScheduleType_c schedule;
} pyramid_data_model_agra_ScheduleStateType_c;

typedef struct pyramid_data_model_agra_SystemScheduleStateType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  int32_t system_schedule_state;
  pyramid_slice_t schedule_state;
} pyramid_data_model_agra_SystemScheduleStateType_c;

typedef struct pyramid_data_model_agra_OpBaseType_c {
  uint8_t has_op_description;
  pyramid_data_model_agra_OpDescriptionType_c op_description;
  uint8_t has_mission_traceability;
  pyramid_data_model_agra_MissionTraceabilityType_c mission_traceability;
  uint8_t has_source;
  int32_t source;
  uint8_t has_schedule;
  pyramid_data_model_agra_ScheduleType_c schedule;
  pyramid_slice_t associated_time;
  pyramid_slice_t data_link_identifier;
  uint8_t has_priority;
  uint32_t priority;
  uint8_t has_qualifying_tags;
  pyramid_data_model_agra_QualifyingTagsType_c qualifying_tags;
  pyramid_slice_t system_id;
  pyramid_slice_t system_schedule_override;
} pyramid_data_model_agra_OpBaseType_c;

typedef struct pyramid_data_model_agra_OpZoneMDT_c {
  uint8_t has_op_description;
  pyramid_data_model_agra_OpDescriptionType_c op_description;  /* from OpBaseType */
  uint8_t has_mission_traceability;
  pyramid_data_model_agra_MissionTraceabilityType_c mission_traceability;  /* from OpBaseType */
  uint8_t has_source;
  int32_t source;  /* from OpBaseType */
  uint8_t has_schedule;
  pyramid_data_model_agra_ScheduleType_c schedule;  /* from OpBaseType */
  pyramid_slice_t associated_time;  /* from OpBaseType */
  pyramid_slice_t data_link_identifier;  /* from OpBaseType */
  uint8_t has_priority;
  uint32_t priority;  /* from OpBaseType */
  uint8_t has_qualifying_tags;
  pyramid_data_model_agra_QualifyingTagsType_c qualifying_tags;  /* from OpBaseType */
  pyramid_slice_t system_id;  /* from OpBaseType */
  pyramid_slice_t system_schedule_override;  /* from OpBaseType */
  pyramid_data_model_agra_OpZoneID_Type_c op_zone_id;
  int32_t category;
  uint8_t has_category_unique_data;
  pyramid_data_model_agra_OpZoneCategoryType_c category_unique_data;
  pyramid_data_model_agra_OpZoneType_c zone;
  uint8_t situational_awareness;
} pyramid_data_model_agra_OpZoneMDT_c;

typedef struct pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c {
  uint8_t has_general;
  int32_t general;
  uint8_t has_hazard;
  int32_t hazard;
  uint8_t has_reference;
  int32_t reference;
  uint8_t has_station;
  int32_t station;
} pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c;

typedef struct pyramid_data_model_agra_KinematicsFixedPositionType_c {
  pyramid_data_model_agra_Point2D_Type_c fixed_point;
  uint8_t has_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c uncertainty;
} pyramid_data_model_agra_KinematicsFixedPositionType_c;

typedef struct pyramid_data_model_agra_OpPointPositionType_c {
  pyramid_data_model_agra_KinematicsFixedPositionType_c position;
  uint8_t has_velocity;
  pyramid_data_model_agra_Velocity2D_UncertaintyType_c velocity;
  uint8_t has_hae_adjustment;
  double hae_adjustment;
} pyramid_data_model_agra_OpPointPositionType_c;

typedef struct pyramid_data_model_agra_OpPointChoiceType_c {
  uint8_t has_point;
  pyramid_data_model_agra_OpPointPositionType_c point;
  uint8_t has_relative_point;
  pyramid_data_model_agra_Point2D_RelativeType_c relative_point;
} pyramid_data_model_agra_OpPointChoiceType_c;

typedef struct pyramid_data_model_agra_AltitudeRadialPairType_c {
  double altitude;
  double radial;
} pyramid_data_model_agra_AltitudeRadialPairType_c;

typedef struct pyramid_data_model_agra_SafeAltitudeType_c {
  pyramid_data_model_agra_Point2D_Type_c center_point_and_esa;
  pyramid_slice_t minimum_safe_altitude;
} pyramid_data_model_agra_SafeAltitudeType_c;

typedef struct pyramid_data_model_agra_SystemConfigurationType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_configuration;
  pyramid_data_model_agra_VehicleConfigurationType_c configuration;
} pyramid_data_model_agra_SystemConfigurationType_c;

typedef struct pyramid_data_model_agra_OpPointBaseType_c {
  uint8_t has_op_description;
  pyramid_data_model_agra_OpDescriptionType_c op_description;  /* from OpBaseType */
  uint8_t has_mission_traceability;
  pyramid_data_model_agra_MissionTraceabilityType_c mission_traceability;  /* from OpBaseType */
  uint8_t has_source;
  int32_t source;  /* from OpBaseType */
  uint8_t has_schedule;
  pyramid_data_model_agra_ScheduleType_c schedule;  /* from OpBaseType */
  pyramid_slice_t associated_time;  /* from OpBaseType */
  pyramid_slice_t data_link_identifier;  /* from OpBaseType */
  uint8_t has_priority;
  uint32_t priority;  /* from OpBaseType */
  uint8_t has_qualifying_tags;
  pyramid_data_model_agra_QualifyingTagsType_c qualifying_tags;  /* from OpBaseType */
  pyramid_slice_t system_id;  /* from OpBaseType */
  pyramid_slice_t system_schedule_override;  /* from OpBaseType */
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
  pyramid_data_model_agra_OpPointChoiceType_c point_choice;
  pyramid_slice_t system_configuration;
  uint8_t has_turn_direction;
  int32_t turn_direction;
  uint8_t has_ingress_constraint;
  pyramid_data_model_agra_AnglePairType_c ingress_constraint;
  uint8_t has_safe_altitude;
  pyramid_data_model_agra_SafeAltitudeType_c safe_altitude;
} pyramid_data_model_agra_OpPointBaseType_c;

typedef struct pyramid_data_model_agra_EmergencyReferenceOpPointType_c {
  pyramid_data_model_agra_OpBaseType_c base;  /* from OpPointBaseType */
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;  /* from OpPointBaseType */
  pyramid_data_model_agra_OpPointChoiceType_c point_choice;  /* from OpPointBaseType */
  pyramid_slice_t system_configuration;  /* from OpPointBaseType */
  uint8_t has_turn_direction;
  int32_t turn_direction;  /* from OpPointBaseType */
  uint8_t has_ingress_constraint;
  pyramid_data_model_agra_AnglePairType_c ingress_constraint;  /* from OpPointBaseType */
  uint8_t has_safe_altitude;
  pyramid_data_model_agra_SafeAltitudeType_c safe_altitude;  /* from OpPointBaseType */
  pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c emergency_reference_category;
} pyramid_data_model_agra_EmergencyReferenceOpPointType_c;

typedef struct pyramid_data_model_agra_OpVolumeMDT_c {
  uint8_t has_op_description;
  pyramid_data_model_agra_OpDescriptionType_c op_description;  /* from OpBaseType */
  uint8_t has_mission_traceability;
  pyramid_data_model_agra_MissionTraceabilityType_c mission_traceability;  /* from OpBaseType */
  uint8_t has_source;
  int32_t source;  /* from OpBaseType */
  uint8_t has_schedule;
  pyramid_data_model_agra_ScheduleType_c schedule;  /* from OpBaseType */
  pyramid_slice_t associated_time;  /* from OpBaseType */
  pyramid_slice_t data_link_identifier;  /* from OpBaseType */
  uint8_t has_priority;
  uint32_t priority;  /* from OpBaseType */
  uint8_t has_qualifying_tags;
  pyramid_data_model_agra_QualifyingTagsType_c qualifying_tags;  /* from OpBaseType */
  pyramid_slice_t system_id;  /* from OpBaseType */
  pyramid_slice_t system_schedule_override;  /* from OpBaseType */
  pyramid_data_model_agra_OpVolumeID_Type_c op_volume_id;
  int32_t category;
  uint8_t has_category_unique_data;
  pyramid_data_model_agra_OpZoneCategoryType_c category_unique_data;
  pyramid_data_model_agra_OpVolumeType_c volume;
  uint8_t situational_awareness;
} pyramid_data_model_agra_OpVolumeMDT_c;

typedef struct pyramid_data_model_agra_OpLineType_c {
  pyramid_data_model_agra_LinePointChoiceType_c point_choice;  /* from LineType */
  int32_t line_projection;  /* from LineType */
  uint8_t has_left_width;
  double left_width;  /* from LineType */
  uint8_t has_right_width;
  double right_width;  /* from LineType */
  uint8_t has_min_altitude;
  double min_altitude;  /* from LineType */
  uint8_t has_max_altitude;
  double max_altitude;  /* from LineType */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from LineType */
  uint8_t has_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c velocity;
} pyramid_data_model_agra_OpLineType_c;

typedef struct pyramid_data_model_agra_OpLineMDT_c {
  uint8_t has_op_description;
  pyramid_data_model_agra_OpDescriptionType_c op_description;  /* from OpBaseType */
  uint8_t has_mission_traceability;
  pyramid_data_model_agra_MissionTraceabilityType_c mission_traceability;  /* from OpBaseType */
  uint8_t has_source;
  int32_t source;  /* from OpBaseType */
  uint8_t has_schedule;
  pyramid_data_model_agra_ScheduleType_c schedule;  /* from OpBaseType */
  pyramid_slice_t associated_time;  /* from OpBaseType */
  pyramid_slice_t data_link_identifier;  /* from OpBaseType */
  uint8_t has_priority;
  uint32_t priority;  /* from OpBaseType */
  uint8_t has_qualifying_tags;
  pyramid_data_model_agra_QualifyingTagsType_c qualifying_tags;  /* from OpBaseType */
  pyramid_slice_t system_id;  /* from OpBaseType */
  pyramid_slice_t system_schedule_override;  /* from OpBaseType */
  pyramid_data_model_agra_OpLineID_Type_c op_line_id;
  int32_t category;
  pyramid_data_model_agra_OpLineType_c line;
} pyramid_data_model_agra_OpLineMDT_c;

typedef struct pyramid_data_model_agra_SlavedNavigationType_c {
  uint8_t has_slaved_to_system_id;
  pyramid_data_model_agra_SystemID_Type_c slaved_to_system_id;
  uint8_t has_slaved_to_service_id;
  pyramid_data_model_agra_ServiceID_Type_c slaved_to_service_id;
  uint8_t has_slaved_to_capability_id;
  pyramid_data_model_agra_CapabilityID_Type_c slaved_to_capability_id;
} pyramid_data_model_agra_SlavedNavigationType_c;

typedef struct pyramid_data_model_agra_MissionPlanNavigationType_c {
  uint8_t has_speed_override;
  double speed_override;
  uint8_t has_altitude_override;
  double altitude_override;
} pyramid_data_model_agra_MissionPlanNavigationType_c;

typedef struct pyramid_data_model_agra_RelativeNavigationType_c {
  pyramid_data_model_agra_Point3D_Type_c reference_point;
  double offset_north;
  double offset_east;
  double offset_down;
} pyramid_data_model_agra_RelativeNavigationType_c;

typedef struct pyramid_data_model_agra_NavigationSourceType_c {
  uint8_t has_mission_plan_navigation;
  pyramid_data_model_agra_MissionPlanNavigationType_c mission_plan_navigation;
  uint8_t has_fixed_navigation;
  pyramid_data_model_agra_Point3D_Type_c fixed_navigation;
  uint8_t has_manual_navigation;
  pyramid_str_t manual_navigation;
  uint8_t has_auto_pilot_navigation;
  int32_t auto_pilot_navigation;
  uint8_t has_relative_navigation;
  pyramid_data_model_agra_RelativeNavigationType_c relative_navigation;
  uint8_t has_slaved_navigation;
  pyramid_data_model_agra_SlavedNavigationType_c slaved_navigation;
} pyramid_data_model_agra_NavigationSourceType_c;

typedef struct pyramid_data_model_agra_NavigationType_c {
  pyramid_data_model_agra_NavigationSourceType_c source;
  uint8_t has_track_error;
  double track_error;
  uint8_t has_altitude_error;
  double altitude_error;
} pyramid_data_model_agra_NavigationType_c;

typedef struct pyramid_data_model_agra_NavigationReportMDT_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  int32_t source;
  int32_t contingency_level;
  pyramid_data_model_agra_EnduranceType_c endurance;
  uint8_t has_navigation;
  pyramid_data_model_agra_NavigationType_c navigation;
} pyramid_data_model_agra_NavigationReportMDT_c;

typedef struct pyramid_data_model_agra_InertialStateType_c {
  pyramid_data_model_agra_Point4D_Type_c position;
  uint8_t has_position_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c position_uncertainty;
  uint8_t has_domain_velocity;
  pyramid_data_model_agra_Velocity3D_Type_c domain_velocity;
  uint8_t has_ground_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c ground_velocity;
  uint8_t has_domain_acceleration;
  pyramid_data_model_agra_Acceleration3D_Type_c domain_acceleration;
  uint8_t has_orientation;
  pyramid_data_model_agra_OrientationType_c orientation;
  uint8_t has_orientation_rate;
  pyramid_data_model_agra_OrientationRateType_c orientation_rate;
  uint8_t has_link16_position_quality;
  uint32_t link16_position_quality;
} pyramid_data_model_agra_InertialStateType_c;

typedef struct pyramid_data_model_agra_PositionReportMDT_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  pyramid_str_t display_name;
  int32_t source;
  int32_t current_operating_domain;
  pyramid_data_model_agra_InertialStateType_c inertial_state;
  uint8_t has_wander_angle;
  double wander_angle;
  uint8_t has_magnetic_heading;
  double magnetic_heading;
} pyramid_data_model_agra_PositionReportMDT_c;

typedef struct pyramid_data_model_agra_MetadataID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MetadataID_Type_c;

typedef struct pyramid_data_model_agra_SystemMetadataPET_c {
} pyramid_data_model_agra_SystemMetadataPET_c;

typedef struct pyramid_data_model_agra_SystemMetadataMDT_c {
  pyramid_data_model_agra_MetadataID_Type_c metadata_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  pyramid_slice_t metadata;
} pyramid_data_model_agra_SystemMetadataMDT_c;

typedef struct pyramid_data_model_agra_SubsystemID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SubsystemID_Type_c;

typedef struct pyramid_data_model_agra_StrengthRangeType_c {
  uint8_t has_minimum;
  uint32_t minimum;
  uint8_t has_maximum;
  uint32_t maximum;
} pyramid_data_model_agra_StrengthRangeType_c;

typedef struct pyramid_data_model_agra_StrengthType_c {
  uint8_t has_strength_value;
  pyramid_data_model_agra_StrengthRangeType_c strength_value;
  uint8_t has_vehicle_count;
  pyramid_data_model_agra_StrengthRangeType_c vehicle_count;
  uint8_t has_percent_tracked;
  pyramid_data_model_agra_PercentRangeType_c percent_tracked;
} pyramid_data_model_agra_StrengthType_c;

typedef struct pyramid_data_model_agra_SystemIdentityType_c {
  uint8_t has_standard;
  pyramid_data_model_agra_StandardIdentityType_c standard;  /* from IdentityType */
  uint8_t has_environment;
  pyramid_data_model_agra_EnvironmentIdentityType_c environment;  /* from IdentityType */
  uint8_t has_platform;
  pyramid_data_model_agra_PlatformIdentityType_c platform;  /* from IdentityType */
  uint8_t has_specific;
  pyramid_data_model_agra_SpecificIdentityType_c specific;  /* from IdentityType */
  pyramid_slice_t emitter;  /* from IdentityType */
  pyramid_slice_t specific_emitter;  /* from IdentityType */
  uint8_t has_specific_facility;
  pyramid_data_model_agra_FacilityIdentificationType_c specific_facility;  /* from IdentityType */
  uint8_t has_specific_vehicle;
  pyramid_data_model_agra_VehicleIdentificationType_c specific_vehicle;  /* from IdentityType */
  uint8_t has_eob;
  pyramid_data_model_agra_EOB_IdentityType_c eob;  /* from IdentityType */
  uint8_t has_weapon;
  pyramid_data_model_agra_StoreType_c weapon;  /* from IdentityType */
  uint8_t has_qualifying_tags;
  pyramid_data_model_agra_QualifyingTagsType_c qualifying_tags;
  pyramid_slice_t associated_time;
} pyramid_data_model_agra_SystemIdentityType_c;

typedef struct pyramid_data_model_agra_PlatformStatusSAM_Type_c {
  uint8_t has_hot_inventory;
  uint32_t hot_inventory;
  uint8_t has_cold_inventory;
  uint32_t cold_inventory;
  uint8_t has_sam_mode;
  int32_t sam_mode;
  uint8_t has_operational_impairment;
  int32_t operational_impairment;
  uint8_t has_communications_impairment;
  int32_t communications_impairment;
  uint8_t has_control_positions;
  uint32_t control_positions;
  uint8_t has_radiation_mode;
  int32_t radiation_mode;
} pyramid_data_model_agra_PlatformStatusSAM_Type_c;

typedef struct pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c {
  uint8_t has_air;
  int32_t air;
  uint8_t has_sea_surface;
  int32_t sea_surface;
  uint8_t has_ground;
  int32_t ground;
  uint8_t has_ew;
  int32_t ew;
} pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c;

typedef struct pyramid_data_model_agra_PlatformFunctionStatusType_c {
  pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c category;
  int32_t status;
} pyramid_data_model_agra_PlatformFunctionStatusType_c;

typedef struct pyramid_data_model_agra_SurfaceRecoveryType_c {
  int32_t flight_deck_status;
  int32_t approach_condition_status;
} pyramid_data_model_agra_SurfaceRecoveryType_c;

typedef struct pyramid_data_model_agra_DatalinkControlType_c {
  int32_t status;
  uint8_t has_network_link_id;
  pyramid_data_model_agra_NetworkLinkID_Type_c network_link_id;
} pyramid_data_model_agra_DatalinkControlType_c;

typedef struct pyramid_data_model_agra_PlatformStatusType_c {
  uint8_t has_bailout_indicator;
  uint8_t bailout_indicator;
  uint8_t has_command_and_control_indicator;
  uint8_t command_and_control_indicator;
  uint8_t has_emergency_indicator;
  uint8_t emergency_indicator;
  pyramid_slice_t function_status;
  uint8_t has_sam_status;
  pyramid_data_model_agra_PlatformStatusSAM_Type_c sam_status;
  uint8_t has_surface_recovery;
  pyramid_data_model_agra_SurfaceRecoveryType_c surface_recovery;
  uint8_t has_surface_status;
  pyramid_data_model_agra_SurfaceRecoveryType_c surface_status;
  pyramid_slice_t datalink_control_status;
} pyramid_data_model_agra_PlatformStatusType_c;

typedef struct pyramid_data_model_agra_VoiceControlType_c {
  uint8_t has_voice_frequency;
  double voice_frequency;
  uint8_t has_voice_channel_a;
  uint32_t voice_channel_a;
  uint8_t has_voice_channel_b;
  uint32_t voice_channel_b;
} pyramid_data_model_agra_VoiceControlType_c;

typedef struct pyramid_data_model_agra_CommAntennaModeType_c {
  pyramid_str_t key;  /* from ForeignKeyType */
  pyramid_str_t system_name;  /* from ForeignKeyType */
} pyramid_data_model_agra_CommAntennaModeType_c;

typedef struct pyramid_data_model_agra_CommModeUsageType_c {
  pyramid_data_model_agra_CommAntennaModeType_c mode_identifier;
  uint8_t has_dropout_threshold;
  double dropout_threshold;
  int32_t concurrent_user_limit;
} pyramid_data_model_agra_CommModeUsageType_c;

typedef struct pyramid_data_model_agra_CommSystemID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_CommSystemID_Type_c;

typedef struct pyramid_data_model_agra_CommSystemUsageType_c {
  pyramid_data_model_agra_CommSystemID_Type_c comm_system_id;
  pyramid_slice_t user_identifier;
  pyramid_slice_t usage;
} pyramid_data_model_agra_CommSystemUsageType_c;

typedef struct pyramid_data_model_agra_SystemCommunicationsType_c {
  int32_t mission_communications_state;
  pyramid_str_t comm_timeout;
  pyramid_str_t pre_planned_comm_return_time;
  pyramid_slice_t comm_usage;
} pyramid_data_model_agra_SystemCommunicationsType_c;

typedef struct pyramid_data_model_agra_SystemStatusMDT_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  int32_t system_state;
  int32_t source;
  uint8_t has_fusion_eligibility;
  int32_t fusion_eligibility;
  pyramid_str_t model;
  uint8_t has_identity;
  pyramid_data_model_agra_SystemIdentityType_c identity;
  pyramid_data_model_agra_SystemCommunicationsType_c communications;
  pyramid_slice_t operator_id;
  pyramid_slice_t subsystem_id;
  pyramid_slice_t capability_id;
  pyramid_slice_t service_id;
  uint8_t has_platform_status;
  pyramid_data_model_agra_PlatformStatusType_c platform_status;
  uint8_t has_voice_control;
  pyramid_data_model_agra_VoiceControlType_c voice_control;
  pyramid_slice_t activity_by;
  uint8_t has_strength;
  pyramid_data_model_agra_StrengthType_c strength;
} pyramid_data_model_agra_SystemStatusMDT_c;

typedef struct pyramid_data_model_agra_SystemDataType_c {
  pyramid_data_model_agra_SystemStatusMDT_c system_status;
  uint8_t has_position;
  pyramid_data_model_agra_PositionReportMDT_c position;
  uint8_t has_navigation;
  pyramid_data_model_agra_NavigationReportMDT_c navigation;
  uint8_t has_metadata;
  pyramid_data_model_agra_SystemMetadataMDT_c metadata;
} pyramid_data_model_agra_SystemDataType_c;

typedef struct pyramid_data_model_agra_EntityMetadataPET_c {
} pyramid_data_model_agra_EntityMetadataPET_c;

typedef struct pyramid_data_model_agra_EntityMetadataMDT_c {
  pyramid_data_model_agra_MetadataID_Type_c metadata_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  pyramid_slice_t metadata;
} pyramid_data_model_agra_EntityMetadataMDT_c;

typedef struct pyramid_data_model_agra_PulseDataID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_PulseDataID_Type_c;

typedef struct pyramid_data_model_agra_EntityRemoveInfoType_c {
  int32_t remove_reason;
  uint8_t has_merged_to_id;
  pyramid_data_model_agra_EntityID_Type_c merged_to_id;
} pyramid_data_model_agra_EntityRemoveInfoType_c;

typedef struct pyramid_data_model_agra_ProductMetadataID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ProductMetadataID_Type_c;

typedef struct pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c;

typedef struct pyramid_data_model_agra_SOB_C2_RecordID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SOB_C2_RecordID_Type_c;

typedef struct pyramid_data_model_agra_EntityExternalType_c {
  uint8_t has_foreign_specific_track_type;
  pyramid_data_model_agra_ForeignKeyType_c foreign_specific_track_type;
  uint8_t has_foreign_track_number;
  pyramid_data_model_agra_ForeignKeyType_c foreign_track_number;
} pyramid_data_model_agra_EntityExternalType_c;

typedef struct pyramid_data_model_agra_EOB_EmitterID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_EOB_EmitterID_Type_c;

typedef struct pyramid_data_model_agra_MeasurementID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MeasurementID_Type_c;

typedef struct pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c;

typedef struct pyramid_data_model_agra_EntityContributorID_ChoiceType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_eob_emitter_id;
  pyramid_data_model_agra_EOB_EmitterID_Type_c eob_emitter_id;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_signal_id;
  pyramid_data_model_agra_SignalID_Type_c signal_id;
  uint8_t has_sob_c2_record_id;
  pyramid_data_model_agra_SOB_C2_RecordID_Type_c sob_c2_record_id;
  uint8_t has_sob_satellite_record_id;
  pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c sob_satellite_record_id;
  uint8_t has_measurement_id;
  pyramid_data_model_agra_MeasurementID_Type_c measurement_id;
} pyramid_data_model_agra_EntityContributorID_ChoiceType_c;

typedef struct pyramid_data_model_agra_SpecificIdentityConfidenceType_c {
  uint32_t specific_type;  /* from SpecificIdentityType */
  int32_t specific_type_category;  /* from SpecificIdentityType */
  uint8_t has_site_type;
  int32_t site_type;  /* from SpecificIdentityType */
  pyramid_str_t specific_type_model;  /* from SpecificIdentityType */
  double confidence;
} pyramid_data_model_agra_SpecificIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_ExerciseIdentityType_c {
  uint8_t has_exercise_identity;
  int32_t exercise_identity;
  uint8_t has_allegiance;
  pyramid_data_model_agra_CountryCodeType_c allegiance;
} pyramid_data_model_agra_ExerciseIdentityType_c;

typedef struct pyramid_data_model_agra_StandardIdentityConfidenceType_c {
  int32_t standard_identity;  /* from StandardIdentityType */
  uint8_t has_allegiance;
  pyramid_data_model_agra_CountryCodeType_c allegiance;  /* from StandardIdentityType */
  double confidence;
  uint8_t has_exercise_identity_data;
  pyramid_data_model_agra_ExerciseIdentityType_c exercise_identity_data;
  uint8_t has_special_identity;
  int32_t special_identity;
} pyramid_data_model_agra_StandardIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_SpecificFacilityIdentityConfidenceType_c {
  pyramid_data_model_agra_FacilityIdentificationType_c facility_identification;
  double confidence;
} pyramid_data_model_agra_SpecificFacilityIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_EmitterIdentityConfidenceType_c {
  pyramid_data_model_agra_EmitterIdentityType_c emitter_category;
  double confidence;
  uint8_t has_matching_parameter_count;
  uint32_t matching_parameter_count;
  pyramid_str_t operator_evaluated;
} pyramid_data_model_agra_EmitterIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_EmitterMultipleType_c {
  pyramid_slice_t emitter_type;
} pyramid_data_model_agra_EmitterMultipleType_c;

typedef struct pyramid_data_model_agra_SpecificEmitterIdentityConfidenceType_c {
  pyramid_data_model_agra_ForeignKeyType_c specific_emitter_key;  /* from SpecificEmitterIdentityType */
  double confidence;
} pyramid_data_model_agra_SpecificEmitterIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_SpecificEmitterMultipleType_c {
  pyramid_slice_t specific_emitter_instance;
} pyramid_data_model_agra_SpecificEmitterMultipleType_c;

typedef struct pyramid_data_model_agra_SpecificVehicleIdentityConfidenceType_c {
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;  /* from VehicleIdentificationType */
  uint8_t has_vehicle_unique_identifier;
  pyramid_data_model_agra_VehicleUniqueIdentifierType_c vehicle_unique_identifier;  /* from VehicleIdentificationType */
  uint8_t has_iff;
  pyramid_data_model_agra_IFF_Type_c iff;  /* from VehicleIdentificationType */
  pyramid_slice_t call_sign;  /* from VehicleIdentificationType */
  pyramid_slice_t radar_cross_section;  /* from VehicleIdentificationType */
  pyramid_slice_t data_link_identifier;  /* from VehicleIdentificationType */
  double confidence;
} pyramid_data_model_agra_SpecificVehicleIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_EOB_IdentityConfidenceType_c {
  pyramid_data_model_agra_EOB_IdentityType_c eob_identity;
  double confidence;
} pyramid_data_model_agra_EOB_IdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_StoreConfidenceType_c {
  pyramid_data_model_agra_StoreType_c weapon_type;
  double confidence;
} pyramid_data_model_agra_StoreConfidenceType_c;

typedef struct pyramid_data_model_agra_StoreMultipleType_c {
  pyramid_slice_t store_instance;
} pyramid_data_model_agra_StoreMultipleType_c;

typedef struct pyramid_data_model_agra_PlatformIdentityConfidenceType_c {
  uint32_t platform_type;  /* from PlatformIdentityType */
  int32_t platform_type_category;  /* from PlatformIdentityType */
  uint8_t has_threat_type;
  int32_t threat_type;  /* from PlatformIdentityType */
  uint8_t has_unit_type;
  int32_t unit_type;  /* from PlatformIdentityType */
  uint8_t has_launch_capability;
  int32_t launch_capability;  /* from PlatformIdentityType */
  uint8_t has_submarine_confidence_level;
  int32_t submarine_confidence_level;  /* from PlatformIdentityType */
  double confidence;
} pyramid_data_model_agra_PlatformIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_EnvironmentIdentityConfidenceType_c {
  int32_t environment;
  uint8_t has_orbit_regime;
  pyramid_data_model_agra_OrbitRegimeType_c orbit_regime;
  uint8_t has_point_track;
  int32_t point_track;
  double confidence;
} pyramid_data_model_agra_EnvironmentIdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_IdentityConfidenceType_c {
  pyramid_slice_t standard;
  pyramid_slice_t environment;
  pyramid_slice_t platform;
  pyramid_slice_t specific;
  pyramid_slice_t emitter;
  pyramid_slice_t specific_emitter;
  pyramid_slice_t specific_vehicle;
  pyramid_slice_t specific_facility;
  pyramid_slice_t eob;
  pyramid_slice_t weapon;
} pyramid_data_model_agra_IdentityConfidenceType_c;

typedef struct pyramid_data_model_agra_EntityFusionSourceType_c {
  pyramid_data_model_agra_EntityContributorID_ChoiceType_c contributor;
  uint8_t fusion_contributor;
  double probability_of_correct_association;
  uint8_t has_contributor_capability_id;
  pyramid_data_model_agra_CapabilityID_Type_c contributor_capability_id;
  uint8_t has_identity;
  pyramid_data_model_agra_IdentityConfidenceType_c identity;
  uint8_t has_source_type;
  int32_t source_type;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  pyramid_str_t timestamp;
} pyramid_data_model_agra_EntityFusionSourceType_c;

typedef struct pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c;

typedef struct pyramid_data_model_agra_EntityCapabilitySourceType_c {
  pyramid_data_model_agra_CapabilityID_Type_c capability_id;
  pyramid_slice_t activity_id;
  uint8_t has_track_number_identifier;
  pyramid_data_model_agra_EntityExternalType_c track_number_identifier;
  pyramid_slice_t correlation_id;
} pyramid_data_model_agra_EntityCapabilitySourceType_c;

typedef struct pyramid_data_model_agra_EOB_RecordID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_EOB_RecordID_Type_c;

typedef struct pyramid_data_model_agra_EntitySourceIdentifierType_c {
  uint8_t has_eob_record_id;
  pyramid_data_model_agra_EOB_RecordID_Type_c eob_record_id;
  uint8_t has_external_identifier;
  pyramid_data_model_agra_EntityExternalType_c external_identifier;
  uint8_t has_fusion;
  pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c fusion;
  uint8_t has_internally_derived_id;
  pyramid_data_model_agra_ID_Type_c internally_derived_id;
  uint8_t has_capability;
  pyramid_data_model_agra_EntityCapabilitySourceType_c capability;
  uint8_t has_product_metadata_id;
  pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c product_metadata_id;
  uint8_t has_operator_id;
  pyramid_data_model_agra_OperatorID_Type_c operator_id;
  uint8_t has_sob_satellite_record_id;
  pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c sob_satellite_record_id;
  uint8_t has_sob_c2_record_id;
  pyramid_data_model_agra_SOB_C2_RecordID_Type_c sob_c2_record_id;
} pyramid_data_model_agra_EntitySourceIdentifierType_c;

typedef struct pyramid_data_model_agra_AssociatedDropRestrictMessageType_c {
  int32_t message_type;
  uint8_t has_associated_id;
  pyramid_data_model_agra_ID_Type_c associated_id;
} pyramid_data_model_agra_AssociatedDropRestrictMessageType_c;

typedef struct pyramid_data_model_agra_DropRestrictionType_c {
  int32_t drop_restriction;
  uint8_t has_associated_message;
  pyramid_data_model_agra_AssociatedDropRestrictMessageType_c associated_message;
} pyramid_data_model_agra_DropRestrictionType_c;

typedef struct pyramid_data_model_agra_CorrelatedEntityID_Type_c {
  pyramid_data_model_agra_ID_Type_c base;  /* from EntityID_Type */
  uint8_t has_source_type;
  int32_t source_type;
} pyramid_data_model_agra_CorrelatedEntityID_Type_c;

typedef struct pyramid_data_model_agra_EntitySourceSpecificDataType_c {
  uint8_t has_fusion_eligibility;
  int32_t fusion_eligibility;
  pyramid_slice_t correlated_entity_id;
  pyramid_slice_t drop_restriction;
  pyramid_slice_t ambiguous_entity_id;
  uint8_t has_track_quality;
  uint32_t track_quality;
  uint8_t has_priority;
  pyramid_data_model_agra_ComparableRankingType_c priority;
  uint8_t has_passive_active_indicator;
  int32_t passive_active_indicator;
} pyramid_data_model_agra_EntitySourceSpecificDataType_c;

typedef struct pyramid_data_model_agra_EntitySourceType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_service_id;
  pyramid_data_model_agra_ServiceID_Type_c service_id;
  uint8_t has_source_specific_data;
  pyramid_data_model_agra_EntitySourceSpecificDataType_c source_specific_data;
  int32_t source_type;
  uint8_t has_source_type_identifier;
  pyramid_data_model_agra_EntitySourceIdentifierType_c source_type_identifier;
  uint8_t has_sensor_type;
  int32_t sensor_type;
} pyramid_data_model_agra_EntitySourceType_c;

typedef struct pyramid_data_model_agra_ActivityActorID_ChoiceType_c {
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_capability_id;
  pyramid_data_model_agra_CapabilityID_Type_c capability_id;
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_ActivityActorID_ChoiceType_c;

typedef struct pyramid_data_model_agra_ActivityAgainstType_c {
  pyramid_data_model_agra_ActivityActorID_ChoiceType_c actor;
  uint32_t activity;
  int32_t activity_category;
  uint8_t has_custom_activity;
  pyramid_data_model_agra_ForeignKeyType_c custom_activity;
} pyramid_data_model_agra_ActivityAgainstType_c;

typedef struct pyramid_data_model_agra_DateTimeSigmaType_c {
  pyramid_str_t date_time;
  pyramid_str_t date_time_sigma;
} pyramid_data_model_agra_DateTimeSigmaType_c;

typedef struct pyramid_data_model_agra_EntityIdentityType_c {
  pyramid_slice_t standard;  /* from IdentityConfidenceType */
  pyramid_slice_t environment;  /* from IdentityConfidenceType */
  pyramid_slice_t platform;  /* from IdentityConfidenceType */
  pyramid_slice_t specific;  /* from IdentityConfidenceType */
  pyramid_slice_t emitter;  /* from IdentityConfidenceType */
  pyramid_slice_t specific_emitter;  /* from IdentityConfidenceType */
  pyramid_slice_t specific_vehicle;  /* from IdentityConfidenceType */
  pyramid_slice_t specific_facility;  /* from IdentityConfidenceType */
  pyramid_slice_t eob;  /* from IdentityConfidenceType */
  pyramid_slice_t weapon;  /* from IdentityConfidenceType */
  uint8_t has_self_reported_identity;
  uint8_t self_reported_identity;
  uint8_t has_difference_indicator;
  uint8_t difference_indicator;
  pyramid_str_t identity_timestamp;
  pyramid_slice_t qualifying_tags;
  pyramid_slice_t associated_time;
} pyramid_data_model_agra_EntityIdentityType_c;

typedef struct pyramid_data_model_agra_FrequencyControlType_c {
  pyramid_data_model_agra_FrequencyRangeType_c frequency_range;
  int32_t control_options;
} pyramid_data_model_agra_FrequencyControlType_c;

typedef struct pyramid_data_model_agra_SignalSummaryType_c {
  pyramid_slice_t modulation;
  pyramid_str_t vendor_specific_modulation;
  uint8_t has_frequency_modulation_variation;
  int32_t frequency_modulation_variation;
  uint8_t has_frequency_average;
  double frequency_average;
  uint8_t has_frequency_min;
  double frequency_min;
  uint8_t has_frequency_max;
  double frequency_max;
  uint8_t has_frequency_range_control_options;
  int32_t frequency_range_control_options;
  uint8_t has_pri_type;
  int32_t pri_type;
  pyramid_str_t pri_average;
  pyramid_str_t pri_min;
  pyramid_str_t pri_max;
  pyramid_str_t pulse_width_average;
  pyramid_str_t pulse_width_min;
  pyramid_str_t pulse_width_max;
  uint8_t has_signal_bandwidth;
  pyramid_data_model_agra_FrequencyRangeType_c signal_bandwidth;
  uint8_t has_amplitude_average;
  double amplitude_average;
  uint8_t has_track_mode;
  int32_t track_mode;
  uint8_t has_urgency;
  int32_t urgency;
  pyramid_str_t wartime_reserve;
  uint8_t has_location_category;
  int32_t location_category;
  uint8_t has_type_of_beam;
  int32_t type_of_beam;
  uint8_t has_beam_persistence;
  int32_t beam_persistence;
  pyramid_slice_t frequency_coverage;
} pyramid_data_model_agra_SignalSummaryType_c;

typedef struct pyramid_data_model_agra_EntitySignalSummaryType_c {
  pyramid_data_model_agra_SignalSummaryType_c signal_description;
  pyramid_str_t latest_detection_timestamp;
  uint8_t has_signal_id;
  pyramid_data_model_agra_SignalID_Type_c signal_id;
} pyramid_data_model_agra_EntitySignalSummaryType_c;

typedef struct pyramid_data_model_agra_OrbitalModelType_c {
  int32_t name;
  pyramid_str_t version;
} pyramid_data_model_agra_OrbitalModelType_c;

typedef struct pyramid_data_model_agra_EphemerisOrbitalModelType_c {
  uint8_t has_orbital_model;
  pyramid_data_model_agra_OrbitalModelType_c orbital_model;
  pyramid_str_t step_size;
  uint8_t has_solar_radiation_pressure_coefficient;
  double solar_radiation_pressure_coefficient;
  uint8_t has_ballistic_coefficient;
  double ballistic_coefficient;
} pyramid_data_model_agra_EphemerisOrbitalModelType_c;

typedef struct pyramid_data_model_agra_OrbitalSingleVectorParametersType_c {
  uint8_t has_area;
  double area;
  uint8_t has_mass;
  double mass;
  uint8_t has_orbital_model;
  pyramid_data_model_agra_EphemerisOrbitalModelType_c orbital_model;
} pyramid_data_model_agra_OrbitalSingleVectorParametersType_c;

typedef struct pyramid_data_model_agra_EntityMDT_c {
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  pyramid_data_model_agra_DateTimeSigmaType_c creation_timestamp;
  pyramid_data_model_agra_EntitySourceType_c source;
  int32_t entity_status;
  uint8_t has_operational_status;
  int32_t operational_status;
  pyramid_data_model_agra_EntityIdentityType_c identity;
  pyramid_slice_t site_entity_id;
  uint8_t has_mobility;
  int32_t mobility;
  uint8_t has_kinematics;
  pyramid_data_model_agra_KinematicsType_c kinematics;
  uint8_t has_estimated_kinematics;
  pyramid_data_model_agra_KinematicsType_c estimated_kinematics;
  uint8_t has_down_location;
  pyramid_data_model_agra_Point2D_Type_c down_location;
  uint8_t has_platform_status;
  pyramid_data_model_agra_PlatformStatusType_c platform_status;
  pyramid_slice_t signal_summary;
  pyramid_slice_t pulse_data_id;
  pyramid_slice_t measurement_id;
  uint8_t has_strength;
  pyramid_data_model_agra_StrengthType_c strength;
  pyramid_slice_t activity_against;
  pyramid_slice_t activity_by;
  uint8_t has_endurance;
  pyramid_data_model_agra_EnduranceType_c endurance;
  uint8_t has_voice_control;
  pyramid_data_model_agra_VoiceControlType_c voice_control;
  pyramid_slice_t associated_id;
  uint8_t has_remove_info;
  pyramid_data_model_agra_EntityRemoveInfoType_c remove_info;
  pyramid_slice_t capability_id;
  uint8_t has_orbital_kinematics;
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c orbital_kinematics;
  uint8_t has_orbital_kinematics_parameters;
  pyramid_data_model_agra_OrbitalSingleVectorParametersType_c orbital_kinematics_parameters;
  uint8_t has_estimated_orbital_kinematics;
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c estimated_orbital_kinematics;
} pyramid_data_model_agra_EntityMDT_c;

typedef struct pyramid_data_model_agra_EntityDataType_c {
  pyramid_data_model_agra_EntityMDT_c entity;
  uint8_t has_metadata;
  pyramid_data_model_agra_EntityMetadataMDT_c metadata;
} pyramid_data_model_agra_EntityDataType_c;

typedef struct pyramid_data_model_agra_GeoLocatedStoredObjectType_c {
  uint8_t has_entity;
  pyramid_data_model_agra_EntityDataType_c entity;
  uint8_t has_system;
  pyramid_data_model_agra_SystemDataType_c system;
  uint8_t has_op_point;
  pyramid_data_model_agra_EmergencyReferenceOpPointType_c op_point;
  uint8_t has_op_line;
  pyramid_data_model_agra_OpLineMDT_c op_line;
  uint8_t has_op_zone;
  pyramid_data_model_agra_OpZoneMDT_c op_zone;
  uint8_t has_op_volume;
  pyramid_data_model_agra_OpVolumeMDT_c op_volume;
} pyramid_data_model_agra_GeoLocatedStoredObjectType_c;

typedef struct pyramid_data_model_agra_OpPointReferenceType_c {
  uint8_t has_stored_object;
  pyramid_data_model_agra_GeoLocatedStoredObjectType_c stored_object;
  uint8_t has_stored_object_ref;
  pyramid_data_model_agra_GeoLocatedObjectType_c stored_object_ref;
  uint8_t has_data_link_object;
  pyramid_data_model_agra_DataLinkIdentifierPET_c data_link_object;
} pyramid_data_model_agra_OpPointReferenceType_c;

typedef struct pyramid_data_model_agra_EmergencyReferencePointType_c {
  pyramid_data_model_agra_OpPointReferenceType_c reference;
  uint8_t has_personnel_count;
  pyramid_data_model_agra_StrengthRangeType_c personnel_count;
  pyramid_str_t last_known_position;
} pyramid_data_model_agra_EmergencyReferencePointType_c;

typedef struct pyramid_data_model_agra_Link16HazardType_c {
  uint8_t has_burst;
  int32_t burst;
  uint8_t has_yield;
  int32_t yield;
} pyramid_data_model_agra_Link16HazardType_c;

typedef struct pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c {
  uint8_t has_emergency;
  pyramid_data_model_agra_EmergencyReferencePointType_c emergency;
  uint8_t has_hazard;
  pyramid_data_model_agra_Link16HazardType_c hazard;
} pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c;

typedef struct pyramid_data_model_agra_ConstrainedOpPointType_c {
  pyramid_data_model_agra_OpPointCategoriesType_c category;
  uint8_t has_category_unique_data;
  pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c category_unique_data;
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
} pyramid_data_model_agra_ConstrainedOpPointType_c;

typedef struct pyramid_data_model_agra_ParameterID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ParameterID_Type_c;

typedef struct pyramid_data_model_agra_ParameterValueType_c {
  uint8_t has_value;
  pyramid_str_t value;
  uint8_t has_return_to_default;
  pyramid_str_t return_to_default;
} pyramid_data_model_agra_ParameterValueType_c;

typedef struct pyramid_data_model_agra_ParameterAssertType_c {
  pyramid_data_model_agra_ParameterID_Type_c parameter_id;
  pyramid_data_model_agra_ParameterValueType_c new_value;
  uint8_t has_object_id;
  pyramid_data_model_agra_ID_Type_c object_id;
} pyramid_data_model_agra_ParameterAssertType_c;

typedef struct pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c {
  uint8_t has_effect;
  int32_t effect;
  uint8_t has_action;
  int32_t action;
  uint8_t has_task;
  int32_t task;
  uint8_t has_capability_command;
  int32_t capability_command;
  uint8_t has_response;
  int32_t response;
} pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c;

typedef struct pyramid_data_model_agra_MA_RequirementChoiceType_c {
  uint8_t has_by_type;
  pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c by_type;
  uint8_t has_by_instance;
  pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c by_instance;
} pyramid_data_model_agra_MA_RequirementChoiceType_c;

typedef struct pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c {
  pyramid_data_model_agra_MA_RequirementChoiceType_c requirement;
  pyramid_slice_t applies_to_system_id;
  pyramid_slice_t risk_level;
} pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c;

typedef struct pyramid_data_model_agra_MA_MissionEnvironmentConstraintType_c {
  uint8_t has_constrained_entity;
  pyramid_data_model_agra_ConstrainedEntityType_c constrained_entity;
  uint8_t has_constrained_op_point;
  pyramid_data_model_agra_ConstrainedOpPointType_c constrained_op_point;
  uint8_t has_constrained_op_line;
  pyramid_data_model_agra_ConstrainedOpLineType_c constrained_op_line;
  uint8_t has_constrained_op_zone;
  pyramid_data_model_agra_ConstrainedOpZoneType_c constrained_op_zone;
  uint8_t has_constrained_op_volume;
  pyramid_data_model_agra_ConstrainedOpVolumeType_c constrained_op_volume;
  uint8_t has_system;
  pyramid_data_model_agra_SystemStatusMDT_c system;
  uint8_t has_risk_adjustment;
  pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c risk_adjustment;
  uint8_t has_parameter;
  pyramid_data_model_agra_ParameterAssertType_c parameter;
} pyramid_data_model_agra_MA_MissionEnvironmentConstraintType_c;

typedef struct pyramid_data_model_agra_MA_CapabilityTaxonomyType_c {
  pyramid_slice_t action;
  pyramid_slice_t air_sample;
  pyramid_slice_t amti;
  pyramid_slice_t ao;
  pyramid_slice_t cap;
  pyramid_slice_t cargo_delivery;
  pyramid_slice_t comint;
  pyramid_slice_t comm_relay;
  uint8_t has_comm_terminal;
  int32_t comm_terminal;
  uint8_t has_counter_space;
  int32_t counter_space;
  pyramid_slice_t ea;
  pyramid_slice_t effect;
  pyramid_slice_t esm;
  pyramid_slice_t flight;
  uint8_t has_iff;
  int32_t iff;
  uint8_t has_opaque;
  int32_t opaque;
  pyramid_slice_t orbit_change;
  pyramid_slice_t orbital_surveillance;
  pyramid_slice_t orbital_surveillance_sensor;
  pyramid_slice_t po;
  uint8_t has_radar_altimeter;
  int32_t radar_altimeter;
  pyramid_slice_t refuel;
  pyramid_slice_t response;
  pyramid_slice_t sar;
  pyramid_slice_t smti;
  pyramid_slice_t strike;
  pyramid_slice_t system_deployment;
  pyramid_slice_t tactical_order;
  uint8_t has_weather_radar;
  int32_t weather_radar;
} pyramid_data_model_agra_MA_CapabilityTaxonomyType_c;

typedef struct pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c {
  pyramid_slice_t effect;  /* from MA_RequirementTaxonomyType */
  pyramid_slice_t action;  /* from MA_RequirementTaxonomyType */
  pyramid_slice_t task;  /* from MA_RequirementTaxonomyType */
  pyramid_slice_t capability_command;  /* from MA_RequirementTaxonomyType */
  pyramid_slice_t response;  /* from MA_RequirementTaxonomyType */
  uint8_t has_capability_taxonomy;
  pyramid_data_model_agra_MA_CapabilityTaxonomyType_c capability_taxonomy;
} pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c;

typedef struct pyramid_data_model_agra_MA_RequirementTriggerType_c {
  uint8_t has_rank;
  pyramid_data_model_agra_RankCompareType_c rank;
  uint8_t has_requirement_types;
  pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c requirement_types;
} pyramid_data_model_agra_MA_RequirementTriggerType_c;

typedef struct pyramid_data_model_agra_MA_RequirementFailedTriggerType_c {
  uint8_t has_rank;
  pyramid_data_model_agra_RankCompareType_c rank;  /* from MA_RequirementTriggerType */
  uint8_t has_requirement_types;
  pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c requirement_types;  /* from MA_RequirementTriggerType */
  pyramid_slice_t dropped_trigger;
} pyramid_data_model_agra_MA_RequirementFailedTriggerType_c;

typedef struct pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c {
  uint8_t has_capability_added;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_added;
  uint8_t has_capability_failure;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_failure;
  uint8_t has_comms_lost;
  pyramid_data_model_agra_CommsLostTriggerDataType_c comms_lost;
  uint8_t has_dmpi_over_designation;
  pyramid_str_t dmpi_over_designation;
  uint8_t has_dmpi_under_designation;
  pyramid_str_t dmpi_under_designation;
  uint8_t has_endurance_low;
  pyramid_data_model_agra_EnduranceType_c endurance_low;
  uint8_t has_off_route;
  pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c off_route;
  uint8_t has_proximity_conflict;
  pyramid_str_t proximity_conflict;
  uint8_t has_release_point_outside_lar;
  pyramid_str_t release_point_outside_lar;
  uint8_t has_route_conflict;
  pyramid_str_t route_conflict;
  uint8_t has_route_vulnerability;
  pyramid_data_model_agra_PlanVulnerabilityType_c route_vulnerability;
  uint8_t has_system_state_change;
  pyramid_data_model_agra_SystemStateFilterType_c system_state_change;
  uint8_t has_requirement_added;
  pyramid_data_model_agra_MA_RequirementTriggerType_c requirement_added;
  uint8_t has_requirement_dependency_failed;
  pyramid_str_t requirement_dependency_failed;
  uint8_t has_requirement_dropped;
  pyramid_data_model_agra_MA_RequirementTriggerType_c requirement_dropped;
  uint8_t has_requirement_failed;
  pyramid_data_model_agra_MA_RequirementFailedTriggerType_c requirement_failed;
  uint8_t has_requirement_change;
  pyramid_data_model_agra_MA_RequirementTriggerType_c requirement_change;
  uint8_t has_requirement_timing;
  pyramid_str_t requirement_timing;
  uint8_t has_zone_violation;
  pyramid_data_model_agra_ZoneViolationTriggerDataType_c zone_violation;
  uint8_t has_orbit_conflict;
  pyramid_str_t orbit_conflict;
  uint8_t has_off_planned_orbit;
  pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c off_planned_orbit;
  uint8_t has_spacecraft_endurance_low;
  pyramid_data_model_agra_SatelliteEnduranceType_c spacecraft_endurance_low;
  uint8_t has_spacecraft_proximity_conflict;
  pyramid_str_t spacecraft_proximity_conflict;
  uint8_t has_response_id;
  pyramid_data_model_agra_ResponseID_Type_c response_id;
} pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c;

typedef struct pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c;

typedef struct pyramid_data_model_agra_MA_PlanningByResultTriggerType_c {
  uint8_t has_replan_required;
  pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c replan_required;
  uint8_t has_vulnerability_changed;
  pyramid_data_model_agra_PlanVulnerabilityType_c vulnerability_changed;
  uint8_t has_requirement_unallocated;
  pyramid_data_model_agra_MA_RequirementTriggerType_c requirement_unallocated;
} pyramid_data_model_agra_MA_PlanningByResultTriggerType_c;

typedef struct pyramid_data_model_agra_MA_PlanningTriggerType_c {
  int32_t source;
  uint8_t has_by_case_trigger;
  pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c by_case_trigger;
  uint8_t has_by_result_trigger;
  pyramid_data_model_agra_MA_PlanningByResultTriggerType_c by_result_trigger;
} pyramid_data_model_agra_MA_PlanningTriggerType_c;

typedef struct pyramid_data_model_agra_MissionContingencyAlertID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MissionContingencyAlertID_Type_c;

typedef struct pyramid_data_model_agra_AutonomousPlanningActionID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_AutonomousPlanningActionID_Type_c;

typedef struct pyramid_data_model_agra_MA_ReplanReasonType_c {
  pyramid_data_model_agra_MA_PlanningTriggerType_c trigger;
  uint8_t has_autonomous_planning_action_id;
  pyramid_data_model_agra_AutonomousPlanningActionID_Type_c autonomous_planning_action_id;
  uint8_t has_mission_contingency_alert_id;
  pyramid_data_model_agra_MissionContingencyAlertID_Type_c mission_contingency_alert_id;
} pyramid_data_model_agra_MA_ReplanReasonType_c;

typedef struct pyramid_data_model_agra_PlanningProcessID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_PlanningProcessID_Type_c;

typedef struct pyramid_data_model_agra_MA_PlanInputsCoreType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;
  int32_t plan_initiation;
  int32_t planning_data_source;
  uint8_t has_replan_reason;
  pyramid_data_model_agra_MA_ReplanReasonType_c replan_reason;
  pyramid_slice_t environment_override;
  pyramid_slice_t op_constraint;
  uint8_t has_special_instructions_id;
  pyramid_data_model_agra_FileLocationID_Type_c special_instructions_id;
} pyramid_data_model_agra_MA_PlanInputsCoreType_c;

typedef struct pyramid_data_model_agra_DMPI_AllocationType_c {
  pyramid_data_model_agra_DMPI_ID_Type_c dmpi_id;
  uint8_t has_dmpi_ranking;
  pyramid_data_model_agra_ComparableRankingType_c dmpi_ranking;
  pyramid_slice_t capability_id;
} pyramid_data_model_agra_DMPI_AllocationType_c;

typedef struct pyramid_data_model_agra_AO_CodeType_c {
  pyramid_str_t prf_code;
  pyramid_str_t pim_code;
} pyramid_data_model_agra_AO_CodeType_c;

typedef struct pyramid_data_model_agra_RequirementAllocationDetailsType_c {
  pyramid_slice_t required_dmpi;
  uint8_t has_required_ao_code;
  pyramid_data_model_agra_AO_CodeType_c required_ao_code;
} pyramid_data_model_agra_RequirementAllocationDetailsType_c;

typedef struct pyramid_data_model_agra_MA_RequirementAllocationBaseType_c {
  uint8_t has_locked;
  uint8_t locked;
  uint8_t has_constraint_override;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c constraint_override;
  uint8_t has_other_override;
  pyramid_data_model_agra_RequirementAllocationDetailsType_c other_override;
} pyramid_data_model_agra_MA_RequirementAllocationBaseType_c;

typedef struct pyramid_data_model_agra_MA_TaskAllocationType_c {
  uint8_t has_locked;
  uint8_t locked;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_constraint_override;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c constraint_override;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_other_override;
  pyramid_data_model_agra_RequirementAllocationDetailsType_c other_override;  /* from MA_RequirementAllocationBaseType */
  pyramid_data_model_agra_TaskID_Type_c task_id;
} pyramid_data_model_agra_MA_TaskAllocationType_c;

typedef struct pyramid_data_model_agra_MA_EffectAllocationType_c {
  uint8_t has_locked;
  uint8_t locked;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_constraint_override;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c constraint_override;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_other_override;
  pyramid_data_model_agra_RequirementAllocationDetailsType_c other_override;  /* from MA_RequirementAllocationBaseType */
  pyramid_data_model_agra_EffectID_Type_c effect_id;
} pyramid_data_model_agra_MA_EffectAllocationType_c;

typedef struct pyramid_data_model_agra_MA_ActionAllocationType_c {
  uint8_t has_locked;
  uint8_t locked;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_constraint_override;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c constraint_override;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_other_override;
  pyramid_data_model_agra_RequirementAllocationDetailsType_c other_override;  /* from MA_RequirementAllocationBaseType */
  pyramid_data_model_agra_ActionID_Type_c action_id;
} pyramid_data_model_agra_MA_ActionAllocationType_c;

typedef struct pyramid_data_model_agra_MA_ResponseAllocationType_c {
  uint8_t has_locked;
  uint8_t locked;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_constraint_override;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c constraint_override;  /* from MA_RequirementAllocationBaseType */
  uint8_t has_other_override;
  pyramid_data_model_agra_RequirementAllocationDetailsType_c other_override;  /* from MA_RequirementAllocationBaseType */
  pyramid_data_model_agra_ResponseID_Type_c response_id;
} pyramid_data_model_agra_MA_ResponseAllocationType_c;

typedef struct pyramid_data_model_agra_MA_RequirementAllocationCommandType_c {
  pyramid_slice_t proposed_effect;
  pyramid_slice_t proposed_action;
  pyramid_slice_t proposed_task;
  pyramid_slice_t proposed_response;
} pyramid_data_model_agra_MA_RequirementAllocationCommandType_c;

typedef struct pyramid_data_model_agra_AssociatedRequirementsType_c {
  pyramid_slice_t requirement_id;
} pyramid_data_model_agra_AssociatedRequirementsType_c;

typedef struct pyramid_data_model_agra_RequirementAssociationConstraintType_c {
  uint8_t has_all_or_nothing;
  pyramid_data_model_agra_AssociatedRequirementsType_c all_or_nothing;
  uint8_t has_either_or;
  pyramid_data_model_agra_AssociatedRequirementsType_c either_or;
  uint8_t has_same_system;
  pyramid_data_model_agra_AssociatedRequirementsType_c same_system;
} pyramid_data_model_agra_RequirementAssociationConstraintType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanInputsType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from MA_PlanInputsCoreType */
  int32_t plan_initiation;  /* from MA_PlanInputsCoreType */
  int32_t planning_data_source;  /* from MA_PlanInputsCoreType */
  uint8_t has_replan_reason;
  pyramid_data_model_agra_MA_ReplanReasonType_c replan_reason;  /* from MA_PlanInputsCoreType */
  pyramid_slice_t environment_override;  /* from MA_PlanInputsCoreType */
  pyramid_slice_t op_constraint;  /* from MA_PlanInputsCoreType */
  uint8_t has_special_instructions_id;
  pyramid_data_model_agra_FileLocationID_Type_c special_instructions_id;  /* from MA_PlanInputsCoreType */
  pyramid_slice_t planning_candidate;
  uint8_t has_proposed_requirement_plans;
  pyramid_data_model_agra_MA_RequirementPlanConstraintType_c proposed_requirement_plans;
  uint8_t has_proposed_requirements;
  pyramid_data_model_agra_MA_RequirementAllocationCommandType_c proposed_requirements;
  pyramid_slice_t association_constraint;
  uint8_t results_in_mission_plan;
  uint8_t has_output_plan_type;
  pyramid_data_model_agra_MA_PlanPartsType_c output_plan_type;
} pyramid_data_model_agra_MA_MissionPlanInputsType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanMDT_c {
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  uint8_t has_mission_plan_command_id;
  pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c mission_plan_command_id;
  pyramid_data_model_agra_MA_PlanApplicabilityType_c applicability;
  uint8_t has_window;
  pyramid_data_model_agra_DateTimeRangeType_c window;
  uint8_t has_sub_plans;
  pyramid_data_model_agra_MA_PlansReferenceBaseType_c sub_plans;
  uint8_t has_execution_sequence;
  pyramid_data_model_agra_ExecutionSequenceType_c execution_sequence;
  pyramid_slice_t parent_mission_plan_id;
  pyramid_slice_t subordinate_mission_plan_id;
  uint8_t for_planning_use_only;
  uint8_t has_plan_inputs;
  pyramid_data_model_agra_MA_MissionPlanInputsType_c plan_inputs;
  uint8_t has_remarks;
  pyramid_data_model_agra_RemarksType_c remarks;
} pyramid_data_model_agra_MA_MissionPlanMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MA_MissionPlanMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanMT_c;

typedef struct pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c {
  pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c command_id;
  pyramid_slice_t approval_management_command_id;
  int32_t command_state;
  pyramid_slice_t command;
  uint8_t has_applicability;
  pyramid_data_model_agra_MA_PlanApplicabilityType_c applicability;
} pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c;

typedef struct pyramid_data_model_agra_MA_PlanActivationStatusType_c {
  int32_t plan_activation_command_state;
  pyramid_data_model_agra_MA_PlansReferenceType_c plans;
} pyramid_data_model_agra_MA_PlanActivationStatusType_c;

typedef struct pyramid_data_model_agra_CompletionStatusType_c {
  pyramid_str_t estimated_time_complete;
  uint8_t has_estimated_percent_complete;
  double estimated_percent_complete;
  pyramid_str_t actual_time_started;
  pyramid_str_t actual_time_complete;
  pyramid_str_t description;
} pyramid_data_model_agra_CompletionStatusType_c;

typedef struct pyramid_data_model_agra_IncompleteProcessingType_c {
  pyramid_data_model_agra_ID_Type_c incomplete_item_id;
  int32_t reason;
  pyramid_str_t description;
} pyramid_data_model_agra_IncompleteProcessingType_c;

typedef struct pyramid_data_model_agra_PlanCommandStatusType_c {
  int32_t command_processing_state;
  uint8_t has_command_processing_state_reason;
  pyramid_data_model_agra_CannotComplyType_c command_processing_state_reason;
  int32_t command_status;
  uint8_t has_completion_status;
  pyramid_data_model_agra_CompletionStatusType_c completion_status;
  pyramid_slice_t incomplete_item;
} pyramid_data_model_agra_PlanCommandStatusType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c {
  pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c command_id;
  pyramid_data_model_agra_PlanCommandStatusType_c activation_status;
  pyramid_slice_t activation_command_by_state;
} pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c;

typedef struct pyramid_data_model_agra_MA_PlanActivationStateType_c {
  int32_t activation_state;
  pyramid_data_model_agra_MA_PlansReferenceBaseType_c plans;
} pyramid_data_model_agra_MA_PlanActivationStateType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c {
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  int32_t plan_activation_state;
  pyramid_slice_t sub_plan_activation_state;
} pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanCommandMDT_c {
  pyramid_data_model_agra_MissionPlanCommandID_Type_c command_id;
  uint8_t has_parent_mission_plan_command_id;
  pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c parent_mission_plan_command_id;
  int32_t command_state;
  pyramid_data_model_agra_MA_MissionPlanInputsType_c inputs;
  uint8_t for_planning_use_only;
} pyramid_data_model_agra_MA_MissionPlanCommandMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanCommandMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_MissionPlanCommandMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanCommandMT_c;

typedef struct pyramid_data_model_agra_UnallocatedReasonType_c {
  int32_t reason;  /* from CannotComplyType */
  pyramid_str_t description;  /* from CannotComplyType */
  uint8_t has_associated_id;
  pyramid_data_model_agra_ID_Type_c associated_id;  /* from CannotComplyType */
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
} pyramid_data_model_agra_UnallocatedReasonType_c;

typedef struct pyramid_data_model_agra_MA_PlanReferenceType_c {
  uint8_t has_mission_plan_id;
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  uint8_t has_task_plan_id;
  pyramid_data_model_agra_TaskPlanID_Type_c task_plan_id;
  uint8_t has_orbit_plan_id;
  pyramid_data_model_agra_OrbitPlanID_Type_c orbit_plan_id;
  uint8_t has_orbit_activity_plan_id;
  pyramid_data_model_agra_OrbitActivityPlanID_Type_c orbit_activity_plan_id;
  uint8_t has_route_plan_id;
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  uint8_t has_route_activity_plan_id;
  pyramid_data_model_agra_RouteActivityPlanID_Type_c route_activity_plan_id;
  uint8_t has_comm_schedule_allocation_id;
  pyramid_data_model_agra_CommScheduleAllocationID_Type_c comm_schedule_allocation_id;
  uint8_t has_activity_plan_id;
  pyramid_data_model_agra_ActivityPlanID_Type_c activity_plan_id;
  uint8_t has_effect_plan_id;
  pyramid_data_model_agra_EffectPlanID_Type_c effect_plan_id;
  uint8_t has_action_plan_id;
  pyramid_data_model_agra_ActionPlanID_Type_c action_plan_id;
  uint8_t has_response_plan_id;
  pyramid_data_model_agra_ResponsePlanID_Type_c response_plan_id;
  uint8_t has_op_plan_id;
  pyramid_data_model_agra_MA_OpPlanID_Type_c op_plan_id;
} pyramid_data_model_agra_MA_PlanReferenceType_c;

typedef struct pyramid_data_model_agra_MA_RequirementPlanningResultBaseType_c {
  int32_t planning_result;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_associated_plans;
  pyramid_data_model_agra_MA_PlanReferenceType_c associated_plans;
  pyramid_slice_t not_allocated_reason;
} pyramid_data_model_agra_MA_RequirementPlanningResultBaseType_c;

typedef struct pyramid_data_model_agra_MA_RequirementPlanningResultType_c {
  int32_t planning_result;  /* from MA_RequirementPlanningResultBaseType */
  pyramid_data_model_agra_SystemID_Type_c system_id;  /* from MA_RequirementPlanningResultBaseType */
  uint8_t has_associated_plans;
  pyramid_data_model_agra_MA_PlanReferenceType_c associated_plans;  /* from MA_RequirementPlanningResultBaseType */
  pyramid_slice_t not_allocated_reason;  /* from MA_RequirementPlanningResultBaseType */
  pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c requirement_id;
} pyramid_data_model_agra_MA_RequirementPlanningResultType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c {
  pyramid_data_model_agra_MissionPlanCommandID_Type_c command_id;
  pyramid_data_model_agra_PlanCommandStatusType_c planning_status;
  pyramid_slice_t allocation_result;
  uint8_t has_resulting_plan_identifier;
  pyramid_data_model_agra_MA_PlansReferenceType_c resulting_plan_identifier;
} pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c;

typedef struct pyramid_data_model_agra_MA_ExecutionPlanSetExecutionStateType_c {
  pyramid_data_model_agra_ExecutionPlanSetID_Type_c execution_plan_set_id;
  int32_t execution_plan_set_execution_state;
} pyramid_data_model_agra_MA_ExecutionPlanSetExecutionStateType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanExecutionStateType_c {
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
  int32_t plan_execution_state;
  pyramid_slice_t execution_plan_set_status;
} pyramid_data_model_agra_MA_MissionPlanExecutionStateType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  int32_t source;
  pyramid_slice_t plan_execution_status;
} pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c message_data;
} pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c;

typedef struct pyramid_data_model_agra_ApplicabilityType_c {
  pyramid_slice_t system;
  uint8_t has_requirement_types;
  pyramid_data_model_agra_RequirementTaxonomyType_c requirement_types;
  uint8_t has_constraining_plan;
  pyramid_data_model_agra_ConstrainingPlanPartsType_c constraining_plan;
} pyramid_data_model_agra_ApplicabilityType_c;

typedef struct pyramid_data_model_agra_ScoringProcessID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ScoringProcessID_Type_c;

typedef struct pyramid_data_model_agra_PlanScoringDescriptionType_c {
  pyramid_data_model_agra_ScoringProcessID_Type_c scoring_process_id;
  pyramid_slice_t op_constraint;
  pyramid_slice_t applicable_to;
} pyramid_data_model_agra_PlanScoringDescriptionType_c;

typedef struct pyramid_data_model_agra_PlanScoringProcessType_c {
  pyramid_data_model_agra_ScoringProcessID_Type_c default_scoring_processed_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_PlanScoringProcessType_c;

typedef struct pyramid_data_model_agra_PlanningFunctionID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_PlanningFunctionID_Type_c;

typedef struct pyramid_data_model_agra_PlanningInterfaceType_c {
  int32_t plan_type;
  pyramid_slice_t plan_interface;
} pyramid_data_model_agra_PlanningInterfaceType_c;

typedef struct pyramid_data_model_agra_PlanningInterfacesType_c {
  int32_t plan_type;  /* from PlanningInterfaceType */
  pyramid_slice_t plan_interface;  /* from PlanningInterfaceType */
  int32_t plan_simultaneity;
} pyramid_data_model_agra_PlanningInterfacesType_c;

typedef struct pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c {
  uint8_t has_by_mission_plan;
  pyramid_data_model_agra_MissionPlanActivationSettingType_c by_mission_plan;
  pyramid_slice_t by_sub_plan;
} pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c;

typedef struct pyramid_data_model_agra_PlanningApplicabilitySystemType_c {
  pyramid_slice_t system;  /* from ApplicabilityType */
  uint8_t has_requirement_types;
  pyramid_data_model_agra_RequirementTaxonomyType_c requirement_types;  /* from ApplicabilityType */
  uint8_t has_constraining_plan;
  pyramid_data_model_agra_ConstrainingPlanPartsType_c constraining_plan;  /* from ApplicabilityType */
  uint8_t has_constraining_plan_incorporation;
  int32_t constraining_plan_incorporation;
} pyramid_data_model_agra_PlanningApplicabilitySystemType_c;

typedef struct pyramid_data_model_agra_PlanningDiscoveryBaseType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;
  pyramid_slice_t validation_supported;
  pyramid_slice_t applicable_to;
  pyramid_slice_t supports;
  pyramid_slice_t planning_triggers;
} pyramid_data_model_agra_PlanningDiscoveryBaseType_c;

typedef struct pyramid_data_model_agra_EffectPlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_EffectPlanPartsType_c output;
} pyramid_data_model_agra_EffectPlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_EffectPlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_EffectPlanProcessType_c;

typedef struct pyramid_data_model_agra_ActionPlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_ActionPlanPartsType_c output;
} pyramid_data_model_agra_ActionPlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_ActionPlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_ActionPlanProcessType_c;

typedef struct pyramid_data_model_agra_OrbitPlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_OrbitPlanPartsType_c output;
} pyramid_data_model_agra_OrbitPlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_OrbitPlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_OrbitPlanProcessType_c;

typedef struct pyramid_data_model_agra_RoutePlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_RoutePlanPartsType_c output;
} pyramid_data_model_agra_RoutePlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_RoutePlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_RoutePlanProcessType_c;

typedef struct pyramid_data_model_agra_ActivityPlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_ActivityPlanPartsType_c output;
} pyramid_data_model_agra_ActivityPlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_ActivityPlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_ActivityPlanProcessType_c;

typedef struct pyramid_data_model_agra_TaskPlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_TaskPlanPartsType_c output;
} pyramid_data_model_agra_TaskPlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_TaskPlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_TaskPlanProcessType_c;

typedef struct pyramid_data_model_agra_ResponsePlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_ResponsePlanPartsType_c output;
} pyramid_data_model_agra_ResponsePlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_ResponsePlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_ResponsePlanProcessType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanProcessDescriptionType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t validation_supported;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t applicable_to;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t supports;  /* from PlanningDiscoveryBaseType */
  pyramid_slice_t planning_triggers;  /* from PlanningDiscoveryBaseType */
  pyramid_data_model_agra_MA_PlanPartsType_c output;
} pyramid_data_model_agra_MA_MissionPlanProcessDescriptionType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanProcessType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c default_planning_process_id;
  pyramid_slice_t process;
} pyramid_data_model_agra_MA_MissionPlanProcessType_c;

typedef struct pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c {
  uint8_t has_mission_plan;
  pyramid_data_model_agra_MA_MissionPlanProcessType_c mission_plan;
  uint8_t has_task_plan;
  pyramid_data_model_agra_TaskPlanProcessType_c task_plan;
  uint8_t has_route_plan;
  pyramid_data_model_agra_RoutePlanProcessType_c route_plan;
  uint8_t has_route_activity_plan;
  pyramid_data_model_agra_ActivityPlanProcessType_c route_activity_plan;
  uint8_t has_activity_plan;
  pyramid_data_model_agra_ActivityPlanProcessType_c activity_plan;
  uint8_t has_orbit_plan;
  pyramid_data_model_agra_OrbitPlanProcessType_c orbit_plan;
  uint8_t has_orbit_activity_plan;
  pyramid_data_model_agra_ActivityPlanProcessType_c orbit_activity_plan;
  uint8_t has_effect_plan;
  pyramid_data_model_agra_EffectPlanProcessType_c effect_plan;
  uint8_t has_action_plan_process;
  pyramid_data_model_agra_ActionPlanProcessType_c action_plan_process;
  uint8_t has_response_plan_process;
  pyramid_data_model_agra_ResponsePlanProcessType_c response_plan_process;
} pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionMDT_c {
  pyramid_data_model_agra_PlanningFunctionID_Type_c planning_function_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  pyramid_slice_t planning_interfaces;
  uint8_t has_planning_interface_details;
  pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c planning_interface_details;
  uint8_t has_plan_activation_autonomy_details;
  pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c plan_activation_autonomy_details;
  uint8_t has_scoring_interface_details;
  pyramid_data_model_agra_PlanScoringProcessType_c scoring_interface_details;
} pyramid_data_model_agra_MA_PlanningFunctionMDT_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MA_PlanningFunctionMDT_c message_data;
} pyramid_data_model_agra_MA_PlanningFunctionMT_c;

typedef struct pyramid_data_model_agra_ContingencyPathSpacingType_c {
  uint8_t has_duration;
  pyramid_str_t duration;
  uint8_t has_distance;
  double distance;
  uint8_t has_endpoints;
  pyramid_str_t endpoints;
} pyramid_data_model_agra_ContingencyPathSpacingType_c;

typedef struct pyramid_data_model_agra_ContingencyPathAutonomyType_c {
  int32_t path_type;
  uint8_t allow_usage_of_alternate_landing_site;
  uint32_t number_routes_to_generate;
  pyramid_data_model_agra_ContingencyPathSpacingType_c spacing;
} pyramid_data_model_agra_ContingencyPathAutonomyType_c;

typedef struct pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c;

typedef struct pyramid_data_model_agra_PlanActivationAutonomyType_c {
  uint8_t has_by_mission_plan;
  pyramid_data_model_agra_MissionPlanActivationSettingType_c by_mission_plan;
  uint8_t has_by_sub_plan;
  pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c by_sub_plan;
} pyramid_data_model_agra_PlanActivationAutonomyType_c;

typedef struct pyramid_data_model_agra_CommandBaseType_c {
  pyramid_data_model_agra_CommandID_Type_c command_id;
  int32_t command_state;
} pyramid_data_model_agra_CommandBaseType_c;

typedef struct pyramid_data_model_agra_MA_AutonomousPlanCommandType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c constraining_plans;
  pyramid_slice_t other_system_constraining_plans;
  uint8_t has_output_plan_parts;
  pyramid_data_model_agra_MA_PlanPartsType_c output_plan_parts;
  uint8_t results_in_mission_plan;
  uint8_t for_planning_use_only;
} pyramid_data_model_agra_MA_AutonomousPlanCommandType_c;

typedef struct pyramid_data_model_agra_MA_PlanningAllowedType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;  /* from MA_AutonomousPlanCommandType */
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c constraining_plans;  /* from MA_AutonomousPlanCommandType */
  pyramid_slice_t other_system_constraining_plans;  /* from MA_AutonomousPlanCommandType */
  uint8_t has_output_plan_parts;
  pyramid_data_model_agra_MA_PlanPartsType_c output_plan_parts;  /* from MA_AutonomousPlanCommandType */
  uint8_t results_in_mission_plan;  /* from MA_AutonomousPlanCommandType */
  uint8_t for_planning_use_only;  /* from MA_AutonomousPlanCommandType */
  pyramid_str_t plan_ahead_duration;
} pyramid_data_model_agra_MA_PlanningAllowedType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c {
  uint8_t has_planning_allowed;
  pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c planning_allowed;
  uint8_t has_alert_only;
  pyramid_str_t alert_only;
} pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanningAutonomySettingByResultType_c {
  pyramid_data_model_agra_PlanningByResultTriggerType_c trigger;
  pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c autonomous_action;
} pyramid_data_model_agra_MA_MissionPlanningAutonomySettingByResultType_c;

typedef struct pyramid_data_model_agra_MA_PlanningAllowedEscalationType_c {
  pyramid_data_model_agra_AutonomousPlanningActionID_Type_c autonomous_planning_action_id;
  pyramid_slice_t by_result_setting;
  pyramid_data_model_agra_MA_PlanningAllowedType_c allowed_type;
} pyramid_data_model_agra_MA_PlanningAllowedEscalationType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c {
  uint8_t has_autonomous_planning_action;
  pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c autonomous_planning_action;
  uint8_t has_alert_only;
  pyramid_str_t alert_only;
} pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c;

typedef struct pyramid_data_model_agra_QueryMessageType_c {
  pyramid_slice_t message_type;
  uint8_t has_query;
  pyramid_data_model_agra_QueryPET_c query;
} pyramid_data_model_agra_QueryMessageType_c;

typedef struct pyramid_data_model_agra_RequirementFilterType_c {
  uint8_t has_requirement_type;
  pyramid_data_model_agra_RequirementTaxonomyType_c requirement_type;
  uint8_t has_requirement_target;
  pyramid_data_model_agra_AssetFilterType_c requirement_target;
} pyramid_data_model_agra_RequirementFilterType_c;

typedef struct pyramid_data_model_agra_WeatherReportGridDataType_c {
  double latitude;  /* from Point3D_Type */
  double longitude;  /* from Point3D_Type */
  double altitude;  /* from Point3D_Type */
  uint8_t has_altitude_reference;
  int32_t altitude_reference;  /* from Point3D_Type */
  pyramid_str_t timestamp;  /* from Point3D_Type */
  pyramid_data_model_agra_WeatherAreaDataType_c weather_data;
} pyramid_data_model_agra_WeatherReportGridDataType_c;

typedef struct pyramid_data_model_agra_OperatorLocationOfInterestComparativeType_c {
  pyramid_data_model_agra_PrioritizationListValueType_c prioritization_list;
  int32_t comparator;
} pyramid_data_model_agra_OperatorLocationOfInterestComparativeType_c;

typedef struct pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c {
  uint8_t has_designation;
  pyramid_data_model_agra_DesignationFilterType_c designation;
  pyramid_slice_t characteristics_clause;
  pyramid_slice_t operator_location_of_interest_id;
  pyramid_slice_t applicable_zone;
  pyramid_slice_t orbital_filter;
} pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c;

typedef struct pyramid_data_model_agra_MA_InertialStateType_c {
  pyramid_data_model_agra_Point4D_Type_c position;
  uint8_t has_position_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c position_uncertainty;
  uint8_t has_domain_velocity;
  pyramid_data_model_agra_Velocity3D_Type_c domain_velocity;
  uint8_t has_calibrated_airspeed;
  double calibrated_airspeed;
  uint8_t has_mach;
  double mach;
  uint8_t has_ground_velocity;
  pyramid_data_model_agra_Velocity2D_Type_c ground_velocity;
  uint8_t has_domain_acceleration;
  pyramid_data_model_agra_Acceleration3D_Type_c domain_acceleration;
  uint8_t has_orientation;
  pyramid_data_model_agra_OrientationType_c orientation;
  uint8_t has_orientation_rate;
  pyramid_data_model_agra_OrientationRateType_c orientation_rate;
  uint8_t has_link16_position_quality;
  uint32_t link16_position_quality;
} pyramid_data_model_agra_MA_InertialStateType_c;

typedef struct pyramid_data_model_agra_WayPointPointChoiceType_c {
  uint8_t has_point2_d;
  pyramid_data_model_agra_Point2D_Type_c point2_d;
  uint8_t has_relative_point;
  pyramid_data_model_agra_Point2D_RelativeType_c relative_point;
} pyramid_data_model_agra_WayPointPointChoiceType_c;

typedef struct pyramid_data_model_agra_WayPointType_c {
  pyramid_data_model_agra_WayPointPointChoiceType_c point_choice;
  int32_t waypoint_type;
  uint8_t has_dmpi_point_id;
  pyramid_data_model_agra_DMPI_ID_Type_c dmpi_point_id;
} pyramid_data_model_agra_WayPointType_c;

typedef struct pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c {
  pyramid_data_model_agra_ResponseID_Type_c response_id;
  uint8_t has_entity;
  pyramid_data_model_agra_EntityFilterType_c entity;
  uint8_t has_system;
  pyramid_data_model_agra_SystemFilterType_c system;
  uint8_t has_requirement;
  pyramid_data_model_agra_RequirementFilterType_c requirement;
  uint8_t has_access_assessment;
  pyramid_data_model_agra_AccessAssessmentFilterType_c access_assessment;
  uint8_t has_oloi;
  pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c oloi;
  uint8_t has_response_command_id;
  pyramid_data_model_agra_ResponseID_Type_c response_command_id;
  uint8_t has_any_message;
  pyramid_data_model_agra_QueryMessageType_c any_message;
  uint8_t has_capability_added;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_added;
  uint8_t has_capability_failure;
  pyramid_data_model_agra_CapabilityTaxonomyType_c capability_failure;
  uint8_t has_comms_lost;
  pyramid_data_model_agra_CommsLostTriggerDataType_c comms_lost;
  uint8_t has_endurance_low;
  pyramid_data_model_agra_EnduranceType_c endurance_low;
  uint8_t has_off_route;
  pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c off_route;
  uint8_t has_proximity_conflict;
  pyramid_data_model_agra_EntityFilterType_c proximity_conflict;
  uint8_t has_route_conflict_id;
  pyramid_data_model_agra_RoutePlanID_Type_c route_conflict_id;
  uint8_t has_route_vulnerability;
  pyramid_data_model_agra_PlanVulnerabilityType_c route_vulnerability;
  uint8_t has_system_state_change;
  pyramid_data_model_agra_SystemStateFilterType_c system_state_change;
  uint8_t has_requirement_added;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_added;
  uint8_t has_requirement_dropped;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_dropped;
  uint8_t has_requirement_failed;
  pyramid_data_model_agra_RequirementFailedTriggerType_c requirement_failed;
  uint8_t has_requirement_change;
  pyramid_data_model_agra_RequirementTriggerType_c requirement_change;
  uint8_t has_zone_violation;
  pyramid_data_model_agra_ZoneViolationTriggerDataType_c zone_violation;
  uint8_t has_orbit_conflict_id;
  pyramid_data_model_agra_OrbitPlanID_Type_c orbit_conflict_id;
  uint8_t has_off_planned_orbit;
  pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c off_planned_orbit;
  uint8_t has_vehicle_state;
  pyramid_data_model_agra_MA_InertialStateType_c vehicle_state;
  uint8_t has_weather;
  pyramid_data_model_agra_WeatherReportGridDataType_c weather;
  uint8_t has_way_point;
  pyramid_data_model_agra_WayPointType_c way_point;
  pyramid_str_t time;
} pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c;

typedef struct pyramid_data_model_agra_MA_MissionPlanningAutonomySettingType_c {
  pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c trigger;
  pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c autonomous_planning_response_choice;
  uint8_t has_planning_process_id;
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;
  pyramid_str_t minimum_response_interval;
} pyramid_data_model_agra_MA_MissionPlanningAutonomySettingType_c;

typedef struct pyramid_data_model_agra_PlanInterfaceStateType_c {
  int32_t plan_interface;
  int32_t activation_state;
} pyramid_data_model_agra_PlanInterfaceStateType_c;

typedef struct pyramid_data_model_agra_PlanInterfaceCommandType_c {
  int32_t plan_type;
  pyramid_slice_t plan_interface;
} pyramid_data_model_agra_PlanInterfaceCommandType_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c {
  pyramid_data_model_agra_CommandID_Type_c command_id;  /* from CommandBaseType */
  int32_t command_state;  /* from CommandBaseType */
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t transition_dropped_to_failed;
  pyramid_slice_t planning_interface;
  pyramid_slice_t mission_planning_autonomy;
  uint8_t has_plan_activation_autonomy;
  pyramid_data_model_agra_PlanActivationAutonomyType_c plan_activation_autonomy;
  pyramid_slice_t route_planning_contingency_path;
} pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c message_data;
} pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c;

typedef struct pyramid_data_model_agra_CommandStatusBaseType_c {
  pyramid_data_model_agra_CommandID_Type_c command_id;
  int32_t command_processing_state;
  uint8_t has_command_processing_state_reason;
  pyramid_data_model_agra_CannotComplyType_c command_processing_state_reason;
} pyramid_data_model_agra_CommandStatusBaseType_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c {
  pyramid_data_model_agra_CommandID_Type_c command_id;  /* from CommandStatusBaseType */
  int32_t command_processing_state;  /* from CommandStatusBaseType */
  uint8_t has_command_processing_state_reason;
  pyramid_data_model_agra_CannotComplyType_c command_processing_state_reason;  /* from CommandStatusBaseType */
} pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c message_data;
} pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t transition_dropped_to_failed;
  pyramid_slice_t planning_interface;
  pyramid_slice_t mission_planning_autonomy;
  uint8_t has_plan_activation_autonomy;
  pyramid_data_model_agra_PlanActivationAutonomyType_c plan_activation_autonomy;
  pyramid_slice_t route_planning_contingency_path;
} pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c message_data;
} pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c;

typedef struct pyramid_data_model_agra_OperatorRecommendationType_c {
  pyramid_data_model_agra_SystemID_Type_c operator_system_id;
  int32_t recommended_operator_action;
} pyramid_data_model_agra_OperatorRecommendationType_c;

typedef struct pyramid_data_model_agra_ResponseAlertType_c {
  pyramid_slice_t by_case_trigger;
  pyramid_slice_t operator_recommendation;
} pyramid_data_model_agra_ResponseAlertType_c;

typedef struct pyramid_data_model_agra_RequirementsTemplateID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_RequirementsTemplateID_Type_c;

typedef struct pyramid_data_model_agra_PlanningTriggerType_c {
  int32_t source;
  uint8_t has_by_case_trigger;
  pyramid_data_model_agra_PlanningByCaseTriggerType_c by_case_trigger;
  uint8_t has_by_result_trigger;
  pyramid_data_model_agra_PlanningByResultTriggerType_c by_result_trigger;
} pyramid_data_model_agra_PlanningTriggerType_c;

typedef struct pyramid_data_model_agra_ReplanReasonType_c {
  pyramid_data_model_agra_PlanningTriggerType_c trigger;
  uint8_t has_autonomous_planning_action_id;
  pyramid_data_model_agra_AutonomousPlanningActionID_Type_c autonomous_planning_action_id;
  uint8_t has_mission_contingency_alert_id;
  pyramid_data_model_agra_MissionContingencyAlertID_Type_c mission_contingency_alert_id;
} pyramid_data_model_agra_ReplanReasonType_c;

typedef struct pyramid_data_model_agra_RequirementTaxonomyChoiceType_c {
  uint8_t has_effect;
  int32_t effect;
  uint8_t has_action;
  int32_t action;
  uint8_t has_task;
  int32_t task;
  uint8_t has_capability_command;
  int32_t capability_command;
  uint8_t has_response;
  int32_t response;
} pyramid_data_model_agra_RequirementTaxonomyChoiceType_c;

typedef struct pyramid_data_model_agra_RequirementChoiceType_c {
  uint8_t has_by_type;
  pyramid_data_model_agra_RequirementTaxonomyChoiceType_c by_type;
  uint8_t has_by_instance;
  pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c by_instance;
} pyramid_data_model_agra_RequirementChoiceType_c;

typedef struct pyramid_data_model_agra_RequirementRiskAdjustmentType_c {
  pyramid_data_model_agra_RequirementChoiceType_c requirement;
  pyramid_slice_t applies_to_system_id;
  pyramid_slice_t risk_level;
} pyramid_data_model_agra_RequirementRiskAdjustmentType_c;

typedef struct pyramid_data_model_agra_MissionEnvironmentConstraintType_c {
  uint8_t has_constrained_entity;
  pyramid_data_model_agra_ConstrainedEntityType_c constrained_entity;
  uint8_t has_constrained_op_point;
  pyramid_data_model_agra_ConstrainedOpPointType_c constrained_op_point;
  uint8_t has_constrained_op_line;
  pyramid_data_model_agra_ConstrainedOpLineType_c constrained_op_line;
  uint8_t has_constrained_op_zone;
  pyramid_data_model_agra_ConstrainedOpZoneType_c constrained_op_zone;
  uint8_t has_constrained_op_volume;
  pyramid_data_model_agra_ConstrainedOpVolumeType_c constrained_op_volume;
  uint8_t has_system;
  pyramid_data_model_agra_SystemStatusMDT_c system;
  uint8_t has_risk_adjustment;
  pyramid_data_model_agra_RequirementRiskAdjustmentType_c risk_adjustment;
  uint8_t has_parameter;
  pyramid_data_model_agra_ParameterAssertType_c parameter;
} pyramid_data_model_agra_MissionEnvironmentConstraintType_c;

typedef struct pyramid_data_model_agra_PlanInputsCoreType_c {
  pyramid_data_model_agra_PlanningProcessID_Type_c planning_process_id;
  int32_t plan_initiation;
  int32_t planning_data_source;
  uint8_t has_replan_reason;
  pyramid_data_model_agra_ReplanReasonType_c replan_reason;
  pyramid_slice_t environment_override;
  pyramid_slice_t op_constraint;
  uint8_t has_special_instructions_id;
  pyramid_data_model_agra_FileLocationID_Type_c special_instructions_id;
} pyramid_data_model_agra_PlanInputsCoreType_c;

typedef struct pyramid_data_model_agra_ResponseTemplateType_c {
  pyramid_data_model_agra_RequirementsTemplateID_Type_c requirements_template_id;
  uint8_t has_planning_inputs;
  pyramid_data_model_agra_PlanInputsCoreType_c planning_inputs;
} pyramid_data_model_agra_ResponseTemplateType_c;

typedef struct pyramid_data_model_agra_MA_RuleResponseType_c {
  uint8_t has_requirements_template;
  pyramid_data_model_agra_ResponseTemplateType_c requirements_template;
  uint8_t has_activate_plan;
  pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c activate_plan;
  uint8_t has_generate_alert;
  pyramid_data_model_agra_ResponseAlertType_c generate_alert;
  uint8_t has_do_nothing;
  pyramid_str_t do_nothing;
} pyramid_data_model_agra_MA_RuleResponseType_c;

typedef struct pyramid_data_model_agra_MA_ResponseOptionDetailsType_c {
  uint32_t option_index;
  pyramid_str_t descriptive_label;
  uint8_t has_objective;
  pyramid_data_model_agra_RequirementTaxonomyType_c objective;
  uint8_t continue_evaluation;
  uint8_t enabled;
  pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c trigger;
  pyramid_data_model_agra_MA_RuleResponseType_c response;
  uint8_t has_schedule;
  pyramid_data_model_agra_ScheduleType_c schedule;
} pyramid_data_model_agra_MA_ResponseOptionDetailsType_c;

typedef struct pyramid_data_model_agra_RequirementAllocationConstraintType_c {
  pyramid_slice_t system_id;
  pyramid_slice_t identity;
  pyramid_slice_t capability_id;
} pyramid_data_model_agra_RequirementAllocationConstraintType_c;

typedef struct pyramid_data_model_agra_RequirementAllocationParametersType_c {
  uint8_t has_required_allocation;
  pyramid_data_model_agra_RequirementAllocationConstraintType_c required_allocation;
  uint8_t has_preferred_allocation;
  pyramid_data_model_agra_RequirementAllocationConstraintType_c preferred_allocation;
} pyramid_data_model_agra_RequirementAllocationParametersType_c;

typedef struct pyramid_data_model_agra_RequirementKinematicConstraintsType_c {
  int32_t applicable_object;
  pyramid_slice_t geo;
  pyramid_slice_t orbital;
} pyramid_data_model_agra_RequirementKinematicConstraintsType_c;

typedef struct pyramid_data_model_agra_RequirementConstraintsType_c {
  uint8_t has_rank;
  pyramid_data_model_agra_ComparableRankingType_c rank;
  uint8_t has_interrupt_lower_rank;
  uint8_t interrupt_lower_rank;
  uint8_t has_allocation;
  pyramid_data_model_agra_RequirementAllocationParametersType_c allocation;
  pyramid_slice_t timing;
  pyramid_slice_t kinematic;
  pyramid_slice_t dependency;
  uint8_t has_max_product_dissemination_classification_level;
  pyramid_data_model_agra_DataProductClassificationLevelType_c max_product_dissemination_classification_level;
  uint8_t has_analytic;
  pyramid_data_model_agra_AnalyticConstraintsType_c analytic;
  uint8_t has_acceptable_classification_level;
  pyramid_data_model_agra_SecurityInformationType_c acceptable_classification_level;
  uint8_t has_comms_required;
  uint8_t comms_required;
  uint8_t has_allowed_requirement_types;
  pyramid_data_model_agra_RequirementTaxonomyType_c allowed_requirement_types;
  uint8_t has_excluded_requirement_types;
  pyramid_data_model_agra_RequirementTaxonomyType_c excluded_requirement_types;
  pyramid_str_t constraints_narrative;
  pyramid_slice_t allowed_domain_pairing;
  pyramid_slice_t excluded_domain_pairing;
} pyramid_data_model_agra_RequirementConstraintsType_c;

typedef struct pyramid_data_model_agra_OrbitChangeTaskBaseType_c {
  int32_t capability_type;
} pyramid_data_model_agra_OrbitChangeTaskBaseType_c;

typedef struct pyramid_data_model_agra_OrbitalManeuverDetailsType_c {
  pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c delta_velocity;  /* from OrbitalManeuverDetailsBaseType */
  uint8_t has_delta_velocity_magnitude;
  double delta_velocity_magnitude;  /* from OrbitalManeuverDetailsBaseType */
  uint8_t has_delta_velocity_covariance;
  pyramid_data_model_agra_CovarianceMatrixType_c delta_velocity_covariance;  /* from OrbitalManeuverDetailsBaseType */
  pyramid_str_t duration;  /* from OrbitalManeuverDetailsBaseType */
  uint8_t has_delta_mass;
  double delta_mass;  /* from OrbitalManeuverDetailsBaseType */
  pyramid_str_t start_epoch;
  uint8_t has_maneuver_type;
  int32_t maneuver_type;
} pyramid_data_model_agra_OrbitalManeuverDetailsType_c;

typedef struct pyramid_data_model_agra_BasicManeuverConstraintsType_c {
  double max_capable_delta_velocity;
  double min_delta_velocity_of_interest;
} pyramid_data_model_agra_BasicManeuverConstraintsType_c;

typedef struct pyramid_data_model_agra_ManeuverConstraintsChoiceType_c {
  uint8_t has_basic_maneuver_constraints;
  pyramid_data_model_agra_BasicManeuverConstraintsType_c basic_maneuver_constraints;
  uint8_t has_predicted_maneuver_constraints;
  pyramid_data_model_agra_OrbitalManeuverDetailsType_c predicted_maneuver_constraints;
} pyramid_data_model_agra_ManeuverConstraintsChoiceType_c;

typedef struct pyramid_data_model_agra_ManeuverDetectionType_c {
  pyramid_data_model_agra_ManeuverConstraintsChoiceType_c maneuver_constraints;
  uint8_t has_expected_maneuver_window;
  pyramid_data_model_agra_DateTimeRangeType_c expected_maneuver_window;
  uint8_t has_velocity_metric_resolution;
  double velocity_metric_resolution;
} pyramid_data_model_agra_ManeuverDetectionType_c;

typedef struct pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c;

typedef struct pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c {
  uint8_t has_body_reference;
  pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c body_reference;
  uint8_t has_attitude;
  pyramid_data_model_agra_QuaternionType_c attitude;
  uint8_t has_aspect_span;
  double aspect_span;
} pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c;

typedef struct pyramid_data_model_agra_DistanceResolutionSpecificationType_c {
  double dimension1;
  uint8_t has_dimension2;
  double dimension2;
  uint8_t has_dimension3;
  double dimension3;
} pyramid_data_model_agra_DistanceResolutionSpecificationType_c;

typedef struct pyramid_data_model_agra_ResolvedCharacterizationType_c {
  pyramid_data_model_agra_DistanceResolutionSpecificationType_c resolution;
  pyramid_data_model_agra_DistanceResolutionSpecificationType_c min_window;
  pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c aspect_coverage;
} pyramid_data_model_agra_ResolvedCharacterizationType_c;

typedef struct pyramid_data_model_agra_SizeEstimationType_c {
  int32_t size_data;
  int32_t size_type;
} pyramid_data_model_agra_SizeEstimationType_c;

typedef struct pyramid_data_model_agra_StructureAssessmentType_c {
  uint8_t has_size_estimation;
  pyramid_data_model_agra_SizeEstimationType_c size_estimation;
  uint8_t has_resolved;
  pyramid_data_model_agra_ResolvedCharacterizationType_c resolved;
} pyramid_data_model_agra_StructureAssessmentType_c;

typedef struct pyramid_data_model_agra_StabilityCharacterizationType_c {
  double min_detectable_angular_rate;
  double max_possible_rate;
  uint8_t has_angular_resolution;
  double angular_resolution;
  uint8_t has_structural_resolution;
  double structural_resolution;
} pyramid_data_model_agra_StabilityCharacterizationType_c;

typedef struct pyramid_data_model_agra_ColorPhotometryParamsType_c {
  pyramid_slice_t sensor_spectrum;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_ColorPhotometryParamsType_c;

typedef struct pyramid_data_model_agra_ProductResolutionType_c {
  uint32_t horizontal_pixel_count;
  uint32_t vertical_pixel_count;
  pyramid_slice_t color_depth;
} pyramid_data_model_agra_ProductResolutionType_c;

typedef struct pyramid_data_model_agra_OpticalImageParamsType_c {
  uint8_t has_image_resolution;
  double image_resolution;
  uint8_t has_product_resolution;
  pyramid_data_model_agra_ProductResolutionType_c product_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_OpticalImageParamsType_c;

typedef struct pyramid_data_model_agra_FrequencyParamsType_c {
  uint8_t has_frequency_range;
  pyramid_data_model_agra_FrequencyRangeType_c frequency_range;
  uint8_t has_min_signal_strength;
  double min_signal_strength;
  uint8_t has_rf_frequency_resolution;
  double rf_frequency_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_FrequencyParamsType_c;

typedef struct pyramid_data_model_agra_VisMagParamsType_c {
  uint8_t has_amplitude_resolution;
  double amplitude_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_VisMagParamsType_c;

typedef struct pyramid_data_model_agra_IR_ImageParamsType_c {
  uint8_t has_spectral_frequency_resolution;
  double spectral_frequency_resolution;
  uint8_t has_temperature_resolution;
  double temperature_resolution;
  uint8_t has_ir_image_setting;
  pyramid_data_model_agra_FrequencyRangeType_c ir_image_setting;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_IR_ImageParamsType_c;

typedef struct pyramid_data_model_agra_Narrowband_SOI_ParamsType_c {
  uint8_t has_amplitude_resolution;
  double amplitude_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_Narrowband_SOI_ParamsType_c;

typedef struct pyramid_data_model_agra_MetricParamsType_c {
  uint8_t has_pos_metric_resolution;
  double pos_metric_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_MetricParamsType_c;

typedef struct pyramid_data_model_agra_PhotometryParamsType_c {
  uint8_t has_amplitude_resolution;
  double amplitude_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_PhotometryParamsType_c;

typedef struct pyramid_data_model_agra_RangeResolutionType_c {
  uint8_t has_range;
  double range;
  uint8_t has_cross_range;
  double cross_range;
} pyramid_data_model_agra_RangeResolutionType_c;

typedef struct pyramid_data_model_agra_Wideband_SOI_ParamsType_c {
  uint8_t has_range_resolution;
  pyramid_data_model_agra_RangeResolutionType_c range_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_Wideband_SOI_ParamsType_c;

typedef struct pyramid_data_model_agra_RCS_ParamsType_c {
  uint8_t has_amplitude_resolution;
  double amplitude_resolution;
  pyramid_str_t product_resolution_time;
} pyramid_data_model_agra_RCS_ParamsType_c;

typedef struct pyramid_data_model_agra_CharacterizationChoiceType_c {
  uint8_t has_frequency;
  pyramid_data_model_agra_FrequencyParamsType_c frequency;
  uint8_t has_ir_image;
  pyramid_data_model_agra_IR_ImageParamsType_c ir_image;
  uint8_t has_metric_observations;
  pyramid_data_model_agra_MetricParamsType_c metric_observations;
  uint8_t has_narrowband_soi;
  pyramid_data_model_agra_Narrowband_SOI_ParamsType_c narrowband_soi;
  uint8_t has_optical_image;
  pyramid_data_model_agra_OpticalImageParamsType_c optical_image;
  uint8_t has_rcs;
  pyramid_data_model_agra_RCS_ParamsType_c rcs;
  uint8_t has_vis_mag;
  pyramid_data_model_agra_VisMagParamsType_c vis_mag;
  uint8_t has_wideband_soi;
  pyramid_data_model_agra_Wideband_SOI_ParamsType_c wideband_soi;
  uint8_t has_photometry;
  pyramid_data_model_agra_PhotometryParamsType_c photometry;
  uint8_t has_color_photometry;
  pyramid_data_model_agra_ColorPhotometryParamsType_c color_photometry;
} pyramid_data_model_agra_CharacterizationChoiceType_c;

typedef struct pyramid_data_model_agra_CharacterizationOptionsType_c {
  pyramid_slice_t characterization_type;
  int32_t logical_operator;
  uint8_t has_characterization_product;
  int32_t characterization_product;
} pyramid_data_model_agra_CharacterizationOptionsType_c;

typedef struct pyramid_data_model_agra_IdentificationVerificationType_c {
  uint8_t has_confidence_desired;
  double confidence_desired;
  int32_t any;
  uint8_t has_all;
  int32_t all;
} pyramid_data_model_agra_IdentificationVerificationType_c;

typedef struct pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c {
  int32_t all;
  int32_t any;
} pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c;

typedef struct pyramid_data_model_agra_CharacterizationObjectiveType_c {
  uint8_t has_phenomenology_collection;
  pyramid_data_model_agra_CharacterizationOptionsType_c phenomenology_collection;
  uint8_t has_stability_and_orientation_assessment;
  pyramid_data_model_agra_StabilityCharacterizationType_c stability_and_orientation_assessment;
  uint8_t has_structure_assessment;
  pyramid_data_model_agra_StructureAssessmentType_c structure_assessment;
  uint8_t has_identification_verification;
  pyramid_data_model_agra_IdentificationVerificationType_c identification_verification;
  uint8_t has_operations_changes;
  pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c operations_changes;
} pyramid_data_model_agra_CharacterizationObjectiveType_c;

typedef struct pyramid_data_model_agra_MultiObjectType_c {
  uint8_t cso_clearing;
  double min_separation;
  uint8_t breakup_characterization;
  uint8_t multi_object_verification;
  uint8_t cso_geometric_characterization;
  uint8_t multi_object_metric;
} pyramid_data_model_agra_MultiObjectType_c;

typedef struct pyramid_data_model_agra_DeploymentDetectionType_c {
  uint8_t cso_clearing;  /* from MultiObjectType */
  double min_separation;  /* from MultiObjectType */
  uint8_t breakup_characterization;  /* from MultiObjectType */
  uint8_t multi_object_verification;  /* from MultiObjectType */
  uint8_t cso_geometric_characterization;  /* from MultiObjectType */
  uint8_t multi_object_metric;  /* from MultiObjectType */
  pyramid_data_model_agra_DateTimeRangeType_c expected_deployment_window;
  double max_separation_velocity;
} pyramid_data_model_agra_DeploymentDetectionType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSearchType_c {
  uint8_t has_max_target_speed;
  double max_target_speed;
  uint8_t has_min_target_speed;
  double min_target_speed;
} pyramid_data_model_agra_OrbitalSurveillanceSearchType_c;

typedef struct pyramid_data_model_agra_OrbitAccuracyType_c {
  pyramid_data_model_agra_RTN_PositionSigmaType_c position_sigma;
  pyramid_data_model_agra_RTN_VelocitySigmaType_c velocity_sigma;
  uint8_t has_time_valid;
  pyramid_data_model_agra_DateTimeRangeType_c time_valid;
} pyramid_data_model_agra_OrbitAccuracyType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c {
  uint8_t has_search;
  pyramid_data_model_agra_OrbitalSurveillanceSearchType_c search;
  uint8_t has_orbit_accuracy;
  pyramid_data_model_agra_OrbitAccuracyType_c orbit_accuracy;
  uint8_t has_characterization;
  pyramid_data_model_agra_CharacterizationObjectiveType_c characterization;
  uint8_t has_multi_object;
  pyramid_data_model_agra_MultiObjectType_c multi_object;
  uint8_t has_maneuver_detection;
  pyramid_data_model_agra_ManeuverDetectionType_c maneuver_detection;
  uint8_t has_deployment_detection;
  pyramid_data_model_agra_DeploymentDetectionType_c deployment_detection;
} pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c;

typedef struct pyramid_data_model_agra_ProductNeededByType_c {
  uint8_t has_as_soon_as_possible;
  pyramid_str_t as_soon_as_possible;
  uint8_t has_absolute_time;
  pyramid_str_t absolute_time;
  uint8_t has_relative_to_event_time;
  pyramid_str_t relative_to_event_time;
} pyramid_data_model_agra_ProductNeededByType_c;

typedef struct pyramid_data_model_agra_SensorConstraintsBaseType_c {
  pyramid_slice_t sensor_type;
  pyramid_slice_t eoir_spectrum;
  pyramid_slice_t rf_spectrum;
  pyramid_slice_t sensor_category;
  pyramid_slice_t sensor_system_identity;
  pyramid_slice_t sensor_basing;
  uint8_t has_owner_operator_country;
  pyramid_data_model_agra_CountryCodeType_c owner_operator_country;
} pyramid_data_model_agra_SensorConstraintsBaseType_c;

typedef struct pyramid_data_model_agra_SensorConstraintsType_c {
  pyramid_slice_t sensor_type;  /* from SensorConstraintsBaseType */
  pyramid_slice_t eoir_spectrum;  /* from SensorConstraintsBaseType */
  pyramid_slice_t rf_spectrum;  /* from SensorConstraintsBaseType */
  pyramid_slice_t sensor_category;  /* from SensorConstraintsBaseType */
  pyramid_slice_t sensor_system_identity;  /* from SensorConstraintsBaseType */
  pyramid_slice_t sensor_basing;  /* from SensorConstraintsBaseType */
  uint8_t has_owner_operator_country;
  pyramid_data_model_agra_CountryCodeType_c owner_operator_country;  /* from SensorConstraintsBaseType */
  pyramid_slice_t system_id;
} pyramid_data_model_agra_SensorConstraintsType_c;

typedef struct pyramid_data_model_agra_SensorCountConstraintType_c {
  int32_t min;
  uint8_t has_max;
  int32_t max;
} pyramid_data_model_agra_SensorCountConstraintType_c;

typedef struct pyramid_data_model_agra_AllowableSensorsType_c {
  uint8_t has_sensor_count_constraint;
  pyramid_data_model_agra_SensorCountConstraintType_c sensor_count_constraint;
  uint8_t has_required;
  pyramid_data_model_agra_SensorConstraintsType_c required;
  uint8_t has_allowed;
  pyramid_data_model_agra_SensorConstraintsType_c allowed;
  uint8_t has_excluded;
  pyramid_data_model_agra_SensorConstraintsType_c excluded;
  uint8_t has_min_operating_classification_level;
  pyramid_data_model_agra_SecurityInformationType_c min_operating_classification_level;
  uint8_t has_near_real_time_product;
  uint8_t near_real_time_product;
} pyramid_data_model_agra_AllowableSensorsType_c;

typedef struct pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c {
  pyramid_slice_t special_instructions;
} pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c;

typedef struct pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c {
  uint8_t has_all;
  pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c all;
  uint8_t has_any;
  pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c any;
} pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c {
  int32_t capability_type;
  pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c sub_capability_type;
  uint8_t has_information_objective;
  int32_t information_objective;
  uint8_t has_min_size;
  double min_size;
  uint8_t confirm_object_acquisition;
  uint8_t has_product_needed_by;
  pyramid_data_model_agra_ProductNeededByType_c product_needed_by;
  uint8_t has_sensor_constraints;
  pyramid_data_model_agra_AllowableSensorsType_c sensor_constraints;
  pyramid_slice_t sda_special_instructions;
} pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c;

typedef struct pyramid_data_model_agra_SystemDeploymentTaskBaseType_c {
  int32_t capability_type;
} pyramid_data_model_agra_SystemDeploymentTaskBaseType_c;

typedef struct pyramid_data_model_agra_FileFormatType_c {
  uint8_t has_mime;
  pyramid_str_t mime;
  uint8_t has_non_mime;
  pyramid_data_model_agra_ForeignKeyType_c non_mime;
} pyramid_data_model_agra_FileFormatType_c;

typedef struct pyramid_data_model_agra_ProductOutputType_c {
  pyramid_slice_t product_type;
  pyramid_slice_t processing_type;
  pyramid_data_model_agra_FileFormatType_c product_format;
} pyramid_data_model_agra_ProductOutputType_c;

typedef struct pyramid_data_model_agra_ProductOutputCommandBasicType_c {
  pyramid_slice_t product_type;  /* from ProductOutputType */
  pyramid_slice_t processing_type;  /* from ProductOutputType */
  pyramid_data_model_agra_FileFormatType_c product_format;  /* from ProductOutputType */
  uint8_t has_product_classification;
  pyramid_data_model_agra_SecurityInformationType_c product_classification;
} pyramid_data_model_agra_ProductOutputCommandBasicType_c;

typedef struct pyramid_data_model_agra_ESM_TaskBaseType_c {
  pyramid_slice_t sub_capability_type;
  pyramid_slice_t output;
} pyramid_data_model_agra_ESM_TaskBaseType_c;

typedef struct pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c;

typedef struct pyramid_data_model_agra_FileOutputType_c {
  pyramid_slice_t file_type;
  pyramid_data_model_agra_FileFormatType_c file_format;
} pyramid_data_model_agra_FileOutputType_c;

typedef struct pyramid_data_model_agra_FileNameAndOutputType_c {
  pyramid_str_t name;
  uint8_t has_type;
  pyramid_data_model_agra_FileOutputType_c type;
} pyramid_data_model_agra_FileNameAndOutputType_c;

typedef struct pyramid_data_model_agra_IPv4_ConnectionType_c {
  pyramid_str_t i_pv4_address;
  uint8_t has_i_pv4_source_cidr_mask;
  int32_t i_pv4_source_cidr_mask;
  uint8_t has_i_pv4_protocol_header;
  int32_t i_pv4_protocol_header;
  uint8_t has_i_pv4_port_range_start;
  int32_t i_pv4_port_range_start;
  uint8_t has_i_pv4_port_range_end;
  int32_t i_pv4_port_range_end;
  uint8_t has_i_pv4_ds_field;
  int32_t i_pv4_ds_field;
  uint8_t has_i_pv4_mtu_size;
  int32_t i_pv4_mtu_size;
} pyramid_data_model_agra_IPv4_ConnectionType_c;

typedef struct pyramid_data_model_agra_IPv6_ConnectionType_c {
  pyramid_str_t i_pv6_address;
  uint8_t has_i_pv6_cidr_mask;
  int32_t i_pv6_cidr_mask;
  uint8_t has_i_pv6_protocol_header;
  int32_t i_pv6_protocol_header;
  uint8_t has_i_pv6_port_range_start;
  int32_t i_pv6_port_range_start;
  uint8_t has_i_pv6_port_range_end;
  int32_t i_pv6_port_range_end;
  uint8_t has_i_pv6_ds_field;
  int32_t i_pv6_ds_field;
  uint8_t has_i_pv6_mtu_size;
  int32_t i_pv6_mtu_size;
} pyramid_data_model_agra_IPv6_ConnectionType_c;

typedef struct pyramid_data_model_agra_IP_ConnectionChoiceType_c {
  uint8_t has_i_pv4;
  pyramid_data_model_agra_IPv4_ConnectionType_c i_pv4;
  uint8_t has_i_pv6;
  pyramid_data_model_agra_IPv6_ConnectionType_c i_pv6;
} pyramid_data_model_agra_IP_ConnectionChoiceType_c;

typedef struct pyramid_data_model_agra_VideoEncoderOutputType_c {
  uint8_t has_socket_address;
  pyramid_data_model_agra_IP_ConnectionChoiceType_c socket_address;
  uint8_t has_file;
  pyramid_data_model_agra_FileNameAndOutputType_c file;
} pyramid_data_model_agra_VideoEncoderOutputType_c;

typedef struct pyramid_data_model_agra_UnsignedIntegerMinMaxType_c {
  uint32_t min;
  uint32_t max;
} pyramid_data_model_agra_UnsignedIntegerMinMaxType_c;

typedef struct pyramid_data_model_agra_CropSettingsType_c {
  uint8_t has_origin;
  int32_t origin;
  uint32_t x_position;
  uint32_t y_position;
  uint32_t width;
  uint32_t height;
} pyramid_data_model_agra_CropSettingsType_c;

typedef struct pyramid_data_model_agra_CropType_c {
  uint8_t enable;
  uint8_t has_settings;
  pyramid_data_model_agra_CropSettingsType_c settings;
} pyramid_data_model_agra_CropType_c;

typedef struct pyramid_data_model_agra_VideoEncoderSettingsType_c {
  uint8_t has_encoding_type;
  pyramid_data_model_agra_ForeignKeyType_c encoding_type;
  pyramid_str_t profile;
  uint8_t has_encoding_blocks;
  int32_t encoding_blocks;
  uint8_t has_chroma_subsample;
  int32_t chroma_subsample;
  uint8_t has_frame_rate;
  double frame_rate;
  uint8_t hdr;
  uint8_t has_bit_rate;
  uint32_t bit_rate;
  uint8_t embed_klv_metadata;
  uint8_t embed_platform_metadata;
  uint8_t has_skip_frames;
  uint32_t skip_frames;
  uint8_t has_quant_min_max;
  pyramid_data_model_agra_UnsignedIntegerMinMaxType_c quant_min_max;
  uint8_t has_compression_mode;
  int32_t compression_mode;
  uint8_t has_b_frames;
  uint32_t b_frames;
  uint8_t has_gop_size;
  uint32_t gop_size;
  uint8_t intra_refresh_mode;
  uint8_t has_intra_refresh_percentage;
  double intra_refresh_percentage;
  uint32_t color_bit_depth;
  uint8_t has_crop;
  pyramid_data_model_agra_CropType_c crop;
  uint8_t has_scaling;
  double scaling;
} pyramid_data_model_agra_VideoEncoderSettingsType_c;

typedef struct pyramid_data_model_agra_VideoOutputSettingsType_c {
  uint8_t enable;
  pyramid_slice_t source_channel;
  pyramid_slice_t output;
  pyramid_data_model_agra_VideoEncoderSettingsType_c encoder_settings;
} pyramid_data_model_agra_VideoOutputSettingsType_c;

typedef struct pyramid_data_model_agra_UMID_DataID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_UMID_DataID_Type_c;

typedef struct pyramid_data_model_agra_UMID_VideoID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_UMID_VideoID_Type_c;

typedef struct pyramid_data_model_agra_UMID_AudioID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_UMID_AudioID_Type_c;

typedef struct pyramid_data_model_agra_UMID_SystemID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_UMID_SystemID_Type_c;

typedef struct pyramid_data_model_agra_MISP_PackingPlanType_c {
  pyramid_data_model_agra_MissionID_Type_c mission_id;
  pyramid_str_t platform_designation;
  pyramid_str_t image_source_sensor;
  pyramid_str_t platform_tail_number;
  uint8_t has_airfield_barometric_pressure;
  double airfield_barometric_pressure;
  uint8_t has_air_field_elevation;
  double air_field_elevation;
  pyramid_str_t platform_call_sign;
  uint8_t has_operational_mode;
  int32_t operational_mode;
  pyramid_str_t classification_or_marking_system;
  pyramid_str_t classification_comment;
  uint8_t has_umid_video_id;
  pyramid_data_model_agra_UMID_VideoID_Type_c umid_video_id;
  uint8_t has_umid_audio_id;
  pyramid_data_model_agra_UMID_AudioID_Type_c umid_audio_id;
  uint8_t has_umid_data_id;
  pyramid_data_model_agra_UMID_DataID_Type_c umid_data_id;
  uint8_t has_umid_system_id;
  pyramid_data_model_agra_UMID_SystemID_Type_c umid_system_id;
  pyramid_str_t item_designator;
} pyramid_data_model_agra_MISP_PackingPlanType_c;

typedef struct pyramid_data_model_agra_NITF_PackingPlanPET_c {
} pyramid_data_model_agra_NITF_PackingPlanPET_c;

typedef struct pyramid_data_model_agra_JPEG_WaveletTransformType_c {
  int32_t type;
  uint32_t level;
} pyramid_data_model_agra_JPEG_WaveletTransformType_c;

typedef struct pyramid_data_model_agra_JPEG_SettingsType_c {
  uint8_t has_compression_ratio;
  float compression_ratio;
  uint8_t has_peak_snr;
  float peak_snr;
  uint8_t has_rate_distortion_slope;
  float rate_distortion_slope;
  uint8_t has_wavelet_transform;
  pyramid_data_model_agra_JPEG_WaveletTransformType_c wavelet_transform;
} pyramid_data_model_agra_JPEG_SettingsType_c;

typedef struct pyramid_data_model_agra_PO_ProductOutputCommandImageryType_c {
  pyramid_data_model_agra_ProductOutputType_c base;  /* from ProductOutputCommandBasicType */
  uint8_t has_product_classification;
  pyramid_data_model_agra_SecurityInformationType_c product_classification;  /* from ProductOutputCommandBasicType */
  pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c product_output_id;
  uint8_t has_jpeg_settings;
  pyramid_data_model_agra_JPEG_SettingsType_c jpeg_settings;
  uint8_t has_nitf_packing_plan;
  pyramid_data_model_agra_NITF_PackingPlanPET_c nitf_packing_plan;
  uint8_t has_compression_ratio;
  float compression_ratio;
  uint8_t has_misp_settings;
  pyramid_data_model_agra_MISP_PackingPlanType_c misp_settings;
  uint8_t has_product_rate;
  uint32_t product_rate;
  uint8_t has_interlaced;
  uint8_t interlaced;
  uint8_t has_product_size;
  uint32_t product_size;
  uint8_t has_product_resolution;
  pyramid_data_model_agra_ProductResolutionType_c product_resolution;
  pyramid_slice_t video_output;
} pyramid_data_model_agra_PO_ProductOutputCommandImageryType_c;

typedef struct pyramid_data_model_agra_AngleQuarterPairType_c {
  double min;
  double max;
} pyramid_data_model_agra_AngleQuarterPairType_c;

typedef struct pyramid_data_model_agra_CollectionConstraintsType_c {
  uint8_t has_map_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c map_angle_constraints;
  uint8_t has_grazing_angle_constraints;
  pyramid_data_model_agra_AngleQuarterPairType_c grazing_angle_constraints;
  uint8_t has_look_angle_constraints;
  pyramid_data_model_agra_AnglePairType_c look_angle_constraints;
  uint8_t has_elevation_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c elevation_angle_constraints;
  uint8_t has_slant_range_constraints;
  pyramid_data_model_agra_SlantRangeConstraintsType_c slant_range_constraints;
  uint8_t has_collection_pattern;
  int32_t collection_pattern;
} pyramid_data_model_agra_CollectionConstraintsType_c;

typedef struct pyramid_data_model_agra_GimbalAxisID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_GimbalAxisID_Type_c;

typedef struct pyramid_data_model_agra_GimbalAxisPairType_c {
  pyramid_data_model_agra_GimbalAxisID_Type_c axis_id;
  pyramid_data_model_agra_AnglePairType_c axis;
} pyramid_data_model_agra_GimbalAxisPairType_c;

typedef struct pyramid_data_model_agra_ComponentID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_ComponentID_Type_c;

typedef struct pyramid_data_model_agra_GimbalOrientationPairType_c {
  int32_t reference_frame;
  uint8_t has_component_id;
  pyramid_data_model_agra_ComponentID_Type_c component_id;
  uint8_t has_pitch;
  pyramid_data_model_agra_AnglePairType_c pitch;
  uint8_t has_roll;
  pyramid_data_model_agra_AnglePairType_c roll;
  uint8_t has_yaw;
  pyramid_data_model_agra_AnglePairType_c yaw;
  pyramid_slice_t axis;
} pyramid_data_model_agra_GimbalOrientationPairType_c;

typedef struct pyramid_data_model_agra_OpticalCollectionConstraintsType_c {
  uint8_t has_map_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c map_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_grazing_angle_constraints;
  pyramid_data_model_agra_AngleQuarterPairType_c grazing_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_look_angle_constraints;
  pyramid_data_model_agra_AnglePairType_c look_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_elevation_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c elevation_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_slant_range_constraints;
  pyramid_data_model_agra_SlantRangeConstraintsType_c slant_range_constraints;  /* from CollectionConstraintsType */
  uint8_t has_collection_pattern;
  int32_t collection_pattern;  /* from CollectionConstraintsType */
  uint8_t has_sun_elev_angles;
  pyramid_data_model_agra_AngleHalfPairType_c sun_elev_angles;
  uint8_t has_sun_azimuth_angles;
  pyramid_data_model_agra_AnglePairType_c sun_azimuth_angles;
  uint8_t has_sweep_speed;
  double sweep_speed;
  uint8_t has_gimbal_orientation;
  pyramid_data_model_agra_GimbalOrientationPairType_c gimbal_orientation;
} pyramid_data_model_agra_OpticalCollectionConstraintsType_c;

typedef struct pyramid_data_model_agra_PO_TaskBaseType_c {
  int32_t capability;
  pyramid_slice_t sensor_spectrum;
  uint8_t has_look_at_coords;
  pyramid_data_model_agra_LocatedEllipseType_c look_at_coords;
  pyramid_str_t minimum_niirs;
  pyramid_str_t desired_niirs;
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_OpticalCollectionConstraintsType_c collection_constraints;
  uint8_t has_pair_identifier;
  int32_t pair_identifier;
  pyramid_slice_t output;
} pyramid_data_model_agra_PO_TaskBaseType_c;

typedef struct pyramid_data_model_agra_TacticalOrderTaskBaseType_c {
  int32_t capability_type;
} pyramid_data_model_agra_TacticalOrderTaskBaseType_c;

typedef struct pyramid_data_model_agra_RefuelTaskBaseType_c {
  int32_t capability_type;
} pyramid_data_model_agra_RefuelTaskBaseType_c;

typedef struct pyramid_data_model_agra_COMINT_TaskBaseType_c {
  int32_t sub_capability_type;
  pyramid_slice_t output;
} pyramid_data_model_agra_COMINT_TaskBaseType_c;

typedef struct pyramid_data_model_agra_FrequencyMultiChannelType_c {
  uint32_t num_channels;
  double spacing;
} pyramid_data_model_agra_FrequencyMultiChannelType_c;

typedef struct pyramid_data_model_agra_FrequencySetType_c {
  pyramid_slice_t frequency_range;
  pyramid_slice_t modulation;
  uint8_t has_encrypted;
  uint8_t encrypted;
  uint8_t has_multi_channel;
  pyramid_data_model_agra_FrequencyMultiChannelType_c multi_channel;
  uint8_t has_content;
  int32_t content;
} pyramid_data_model_agra_FrequencySetType_c;

typedef struct pyramid_data_model_agra_CommRelayTaskBaseType_c {
  int32_t capability_type;
  pyramid_slice_t frequency_set;
} pyramid_data_model_agra_CommRelayTaskBaseType_c;

typedef struct pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c {
} pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c;

typedef struct pyramid_data_model_agra_ProductOutputCommandSMTI_Type_c {
  pyramid_data_model_agra_ProductOutputType_c base;  /* from ProductOutputCommandBasicType */
  uint8_t has_product_classification;
  pyramid_data_model_agra_SecurityInformationType_c product_classification;  /* from ProductOutputCommandBasicType */
  uint8_t has_stanag_4607_packing_plan;
  pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c stanag_4607_packing_plan;
} pyramid_data_model_agra_ProductOutputCommandSMTI_Type_c;

typedef struct pyramid_data_model_agra_RadarTaperWeightingFunctionType_c {
  uint8_t has_standard_weighting_function;
  int32_t standard_weighting_function;
  uint8_t has_other_taper;
  pyramid_data_model_agra_ForeignKeyType_c other_taper;
} pyramid_data_model_agra_RadarTaperWeightingFunctionType_c;

typedef struct pyramid_data_model_agra_RadarTaperType_c {
  pyramid_data_model_agra_RadarTaperWeightingFunctionType_c weighting_function;
  uint8_t has_sidelobe_level;
  double sidelobe_level;
} pyramid_data_model_agra_RadarTaperType_c;

typedef struct pyramid_data_model_agra_RadarSpoilType_c {
  int32_t spoil_type;
  float spoil_factor;
} pyramid_data_model_agra_RadarSpoilType_c;

typedef struct pyramid_data_model_agra_RadarSpoilTaperType_c {
  uint8_t has_taper;
  pyramid_data_model_agra_RadarTaperType_c taper;
  uint8_t has_spoil;
  pyramid_data_model_agra_RadarSpoilType_c spoil;
} pyramid_data_model_agra_RadarSpoilTaperType_c;

typedef struct pyramid_data_model_agra_ElectronicProtectionOptionsEnableType_c {
  pyramid_data_model_agra_ForeignKeyType_c electronic_protection_options_key;
  int32_t electronic_protection_option_status;
} pyramid_data_model_agra_ElectronicProtectionOptionsEnableType_c;

typedef struct pyramid_data_model_agra_RadarCollectionOptionsType_c {
  uint8_t has_collection_policy;
  int32_t collection_policy;
  uint8_t has_force_rx_spoil_taper;
  pyramid_data_model_agra_RadarSpoilTaperType_c force_rx_spoil_taper;
  uint8_t has_force_tx_spoil_taper;
  pyramid_data_model_agra_RadarSpoilTaperType_c force_tx_spoil_taper;
  uint8_t has_concurrent_operation_accepted;
  uint8_t concurrent_operation_accepted;
  pyramid_slice_t electronic_protection_options_override;
} pyramid_data_model_agra_RadarCollectionOptionsType_c;

typedef struct pyramid_data_model_agra_RangeDopplerResolutionType_c {
  pyramid_data_model_agra_RangeResolutionType_c range_resolution;
  uint8_t has_doppler_resolution;
  double doppler_resolution;
} pyramid_data_model_agra_RangeDopplerResolutionType_c;

typedef struct pyramid_data_model_agra_HRR_ChipSizeType_c {
  uint32_t range_size;
  uint32_t doppler_size;
} pyramid_data_model_agra_HRR_ChipSizeType_c;

typedef struct pyramid_data_model_agra_HRR_OptionsType_c {
  uint8_t has_hrr_peak_scatterers;
  uint32_t hrr_peak_scatterers;
  uint8_t has_chip_size;
  pyramid_data_model_agra_HRR_ChipSizeType_c chip_size;
} pyramid_data_model_agra_HRR_OptionsType_c;

typedef struct pyramid_data_model_agra_SMTI_CollectionOptionsType_c {
  uint8_t has_collection_policy;
  int32_t collection_policy;  /* from RadarCollectionOptionsType */
  uint8_t has_force_rx_spoil_taper;
  pyramid_data_model_agra_RadarSpoilTaperType_c force_rx_spoil_taper;  /* from RadarCollectionOptionsType */
  uint8_t has_force_tx_spoil_taper;
  pyramid_data_model_agra_RadarSpoilTaperType_c force_tx_spoil_taper;  /* from RadarCollectionOptionsType */
  uint8_t has_concurrent_operation_accepted;
  uint8_t concurrent_operation_accepted;  /* from RadarCollectionOptionsType */
  pyramid_slice_t electronic_protection_options_override;  /* from RadarCollectionOptionsType */
  uint8_t has_tracking;
  uint8_t tracking;
  uint8_t has_fti;
  uint8_t fti;
  uint8_t has_resolution;
  pyramid_data_model_agra_RangeDopplerResolutionType_c resolution;
  uint8_t has_signal_to_noise_ratio;
  double signal_to_noise_ratio;
  uint8_t has_hrr_options;
  pyramid_data_model_agra_HRR_OptionsType_c hrr_options;
  uint8_t has_coarps_smti_collection_option;
  int32_t coarps_smti_collection_option;
} pyramid_data_model_agra_SMTI_CollectionOptionsType_c;

typedef struct pyramid_data_model_agra_FalseAlarmType_c {
  uint8_t has_cfar_threshold;
  double cfar_threshold;
  uint8_t has_false_alarm_level;
  uint32_t false_alarm_level;
} pyramid_data_model_agra_FalseAlarmType_c;

typedef struct pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c {
  uint8_t has_mtiirs;
  pyramid_str_t mtiirs;
  uint8_t has_circular_error_probable90;
  double circular_error_probable90;
} pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c;

typedef struct pyramid_data_model_agra_PositionLocationUncertaintyType_c {
  pyramid_data_model_agra_Point3D_Type_c location;
  uint8_t has_uncertainty;
  pyramid_data_model_agra_UncertaintyType_c uncertainty;
} pyramid_data_model_agra_PositionLocationUncertaintyType_c;

typedef struct pyramid_data_model_agra_SpeedRangeType_c {
  double min;
  double max;
} pyramid_data_model_agra_SpeedRangeType_c;

typedef struct pyramid_data_model_agra_EmconERP_Type_c {
  uint8_t has_maximum_radiated_erp;
  double maximum_radiated_erp;
  uint8_t has_radiate_full_power;
  uint8_t radiate_full_power;
} pyramid_data_model_agra_EmconERP_Type_c;

typedef struct pyramid_data_model_agra_EmconOverrideType_c {
  uint8_t has_emcon_level;
  int32_t emcon_level;
  uint8_t has_foreign_level;
  pyramid_data_model_agra_ForeignKeyType_c foreign_level;
} pyramid_data_model_agra_EmconOverrideType_c;

typedef struct pyramid_data_model_agra_EmconConstraintType_c {
  uint8_t has_default_emcon_override;
  pyramid_data_model_agra_EmconOverrideType_c default_emcon_override;
  uint8_t has_erp_selection;
  pyramid_data_model_agra_EmconERP_Type_c erp_selection;
} pyramid_data_model_agra_EmconConstraintType_c;

typedef struct pyramid_data_model_agra_SMTI_CollectionConstraintsType_c {
  uint8_t has_map_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c map_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_grazing_angle_constraints;
  pyramid_data_model_agra_AngleQuarterPairType_c grazing_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_look_angle_constraints;
  pyramid_data_model_agra_AnglePairType_c look_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_elevation_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c elevation_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_slant_range_constraints;
  pyramid_data_model_agra_SlantRangeConstraintsType_c slant_range_constraints;  /* from CollectionConstraintsType */
  uint8_t has_collection_pattern;
  int32_t collection_pattern;  /* from CollectionConstraintsType */
  uint8_t has_quality;
  pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c quality;
  uint8_t has_target_speed;
  pyramid_data_model_agra_SpeedRangeType_c target_speed;
  uint8_t has_target_rcs;
  double target_rcs;
  uint8_t has_false_alarm;
  pyramid_data_model_agra_FalseAlarmType_c false_alarm;
  uint8_t has_probability_of_detection;
  double probability_of_detection;
  uint8_t has_emcon;
  pyramid_data_model_agra_EmconConstraintType_c emcon;
  uint8_t has_ownship_location_constraint;
  pyramid_data_model_agra_PositionLocationUncertaintyType_c ownship_location_constraint;
} pyramid_data_model_agra_SMTI_CollectionConstraintsType_c;

typedef struct pyramid_data_model_agra_SMTI_TaskBaseType_c {
  int32_t capability_type;
  uint8_t has_sub_capability_type;
  int32_t sub_capability_type;
  uint8_t has_look_at_coords;
  pyramid_data_model_agra_LocatedEllipseType_c look_at_coords;
  uint8_t has_collection_options;
  pyramid_data_model_agra_SMTI_CollectionOptionsType_c collection_options;
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_SMTI_CollectionConstraintsType_c collection_constraints;
  pyramid_slice_t output;
} pyramid_data_model_agra_SMTI_TaskBaseType_c;

typedef struct pyramid_data_model_agra_AirSampleTaskBaseType_c {
  int32_t capability_type;
  pyramid_slice_t output;
} pyramid_data_model_agra_AirSampleTaskBaseType_c;

typedef struct pyramid_data_model_agra_AO_TaskBaseType_c {
  int32_t capability_type;
  uint8_t has_supported_code;
  pyramid_data_model_agra_AO_CodeType_c supported_code;
  uint8_t has_emission_constraints;
  pyramid_data_model_agra_OpticalCollectionConstraintsType_c emission_constraints;
} pyramid_data_model_agra_AO_TaskBaseType_c;

typedef struct pyramid_data_model_agra_SupportedResolutionID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SupportedResolutionID_Type_c;

typedef struct pyramid_data_model_agra_SAR_CollectionOptionsType_c {
  uint8_t has_collection_policy;
  int32_t collection_policy;  /* from RadarCollectionOptionsType */
  uint8_t has_force_rx_spoil_taper;
  pyramid_data_model_agra_RadarSpoilTaperType_c force_rx_spoil_taper;  /* from RadarCollectionOptionsType */
  uint8_t has_force_tx_spoil_taper;
  pyramid_data_model_agra_RadarSpoilTaperType_c force_tx_spoil_taper;  /* from RadarCollectionOptionsType */
  uint8_t has_concurrent_operation_accepted;
  uint8_t concurrent_operation_accepted;  /* from RadarCollectionOptionsType */
  pyramid_slice_t electronic_protection_options_override;  /* from RadarCollectionOptionsType */
  uint8_t has_atr;
  uint8_t atr;
  uint8_t has_fti;
  uint8_t fti;
  uint8_t has_atr_number_of_targets;
  uint32_t atr_number_of_targets;
  uint8_t has_commanded_resolution;
  double commanded_resolution;
  uint8_t has_supported_resolution_id;
  pyramid_data_model_agra_SupportedResolutionID_Type_c supported_resolution_id;
  uint8_t has_coarps_sar_collection_option;
  int32_t coarps_sar_collection_option;
  uint8_t has_band;
  int32_t band;
} pyramid_data_model_agra_SAR_CollectionOptionsType_c;

typedef struct pyramid_data_model_agra_SAR_WaveformType_c {
  uint8_t has_waveform_type;
  int32_t waveform_type;
  uint8_t has_foreign_waveform;
  pyramid_data_model_agra_ForeignKeyType_c foreign_waveform;
} pyramid_data_model_agra_SAR_WaveformType_c;

typedef struct pyramid_data_model_agra_ProductOutputCommandImageryType_c {
  pyramid_data_model_agra_ProductOutputType_c base;  /* from ProductOutputCommandBasicType */
  uint8_t has_product_classification;
  pyramid_data_model_agra_SecurityInformationType_c product_classification;  /* from ProductOutputCommandBasicType */
  uint8_t has_jpeg_settings;
  pyramid_data_model_agra_JPEG_SettingsType_c jpeg_settings;
  uint8_t has_nitf_packing_plan;
  pyramid_data_model_agra_NITF_PackingPlanPET_c nitf_packing_plan;
} pyramid_data_model_agra_ProductOutputCommandImageryType_c;

typedef struct pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c {
  pyramid_str_t desired_niirs;
  pyramid_str_t minimum_niirs;
  uint8_t has_minimum_resolution;
  double minimum_resolution;
} pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c;

typedef struct pyramid_data_model_agra_SAR_CollectionConstraintsType_c {
  uint8_t has_map_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c map_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_grazing_angle_constraints;
  pyramid_data_model_agra_AngleQuarterPairType_c grazing_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_look_angle_constraints;
  pyramid_data_model_agra_AnglePairType_c look_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_elevation_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c elevation_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_slant_range_constraints;
  pyramid_data_model_agra_SlantRangeConstraintsType_c slant_range_constraints;  /* from CollectionConstraintsType */
  uint8_t has_collection_pattern;
  int32_t collection_pattern;  /* from CollectionConstraintsType */
  uint8_t has_quality;
  pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c quality;
  uint8_t has_continuous_spot_angle;
  double continuous_spot_angle;
  uint8_t has_altitude_constraints;
  pyramid_data_model_agra_AltitudeRangeType_c altitude_constraints;
  pyramid_str_t maximum_mapping_time;
  uint8_t has_az_subset_fraction;
  double az_subset_fraction;
  uint8_t has_los_buffer_angle;
  double los_buffer_angle;
  uint8_t has_num_images_azimuth;
  uint32_t num_images_azimuth;
  uint8_t has_num_images_range;
  uint32_t num_images_range;
  uint8_t has_percent_azimuth_collection;
  double percent_azimuth_collection;
  uint8_t has_heading_deviation_tolerance;
  double heading_deviation_tolerance;
  uint8_t has_cross_track_deviation_tolerance;
  double cross_track_deviation_tolerance;
  uint8_t has_emcon;
  pyramid_data_model_agra_EmconConstraintType_c emcon;
  uint8_t has_map_gain_adjustment;
  double map_gain_adjustment;
  uint8_t has_ownship_location_constraint;
  pyramid_data_model_agra_PositionLocationUncertaintyType_c ownship_location_constraint;
} pyramid_data_model_agra_SAR_CollectionConstraintsType_c;

typedef struct pyramid_data_model_agra_SAR_TaskBaseType_c {
  int32_t capability_type;
  int32_t sub_capability_type;
  uint8_t has_look_at_coords;
  pyramid_data_model_agra_LocatedEllipseType_c look_at_coords;
  uint8_t has_collection_options;
  pyramid_data_model_agra_SAR_CollectionOptionsType_c collection_options;
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_SAR_CollectionConstraintsType_c collection_constraints;
  uint8_t has_desired_waveform;
  pyramid_data_model_agra_SAR_WaveformType_c desired_waveform;
  uint8_t has_pair_identifier;
  int32_t pair_identifier;
  pyramid_slice_t output;
} pyramid_data_model_agra_SAR_TaskBaseType_c;

typedef struct pyramid_data_model_agra_WeaponeeringTargetInfoType_c {
  uint8_t has_target_hardness;
  int32_t target_hardness;
  uint8_t has_target_class;
  uint32_t target_class;
} pyramid_data_model_agra_WeaponeeringTargetInfoType_c;

typedef struct pyramid_data_model_agra_WeaponeeringStoreType_c {
  uint32_t store_type;  /* from StoreType */
  int32_t store_category;  /* from StoreType */
  uint8_t has_store_type_variant;
  pyramid_data_model_agra_ForeignKeyType_c store_type_variant;  /* from StoreType */
  uint32_t store_quantity;
} pyramid_data_model_agra_WeaponeeringStoreType_c;

typedef struct pyramid_data_model_agra_TargetFinalApproachType_c {
  uint8_t has_altitude;
  double altitude;
  uint8_t has_range;
  double range;
} pyramid_data_model_agra_TargetFinalApproachType_c;

typedef struct pyramid_data_model_agra_AzElReferenceType_c {
  int32_t reference;
  uint8_t has_azimuth;
  double azimuth;
  uint8_t has_elevation;
  double elevation;
} pyramid_data_model_agra_AzElReferenceType_c;

typedef struct pyramid_data_model_agra_ApproachAngleType_c {
  uint8_t has_az_el;
  pyramid_data_model_agra_AzElReferenceType_c az_el;
  uint8_t has_relative;
  pyramid_data_model_agra_UnitVectorType_c relative;
} pyramid_data_model_agra_ApproachAngleType_c;

typedef struct pyramid_data_model_agra_ApproachConditionsType_c {
  uint8_t has_flight_mode;
  int32_t flight_mode;
  uint8_t has_range_on_heading;
  double range_on_heading;
  uint8_t has_approach_angle;
  pyramid_data_model_agra_ApproachAngleType_c approach_angle;
  uint8_t has_target_final_approach;
  pyramid_data_model_agra_TargetFinalApproachType_c target_final_approach;
} pyramid_data_model_agra_ApproachConditionsType_c;

typedef struct pyramid_data_model_agra_BodyFaceType_c {
  int32_t desired_face;
  uint8_t has_radial_offset;
  double radial_offset;
  uint8_t has_theta;
  double theta;
} pyramid_data_model_agra_BodyFaceType_c;

typedef struct pyramid_data_model_agra_OffsetLocationType_c {
  float offset_x;
  float offset_y;
  float offset_z;
  uint8_t has_location_key;
  pyramid_data_model_agra_ForeignKeyType_c location_key;
} pyramid_data_model_agra_OffsetLocationType_c;

typedef struct pyramid_data_model_agra_OffsetLocationErrorType_c {
  float offset_x;  /* from OffsetLocationType */
  float offset_y;  /* from OffsetLocationType */
  float offset_z;  /* from OffsetLocationType */
  uint8_t has_location_key;
  pyramid_data_model_agra_ForeignKeyType_c location_key;  /* from OffsetLocationType */
  uint8_t has_cep;
  double cep;
  uint8_t has_confidence;
  double confidence;
} pyramid_data_model_agra_OffsetLocationErrorType_c;

typedef struct pyramid_data_model_agra_ImpactPointType_c {
  uint8_t has_body_location;
  pyramid_data_model_agra_OffsetLocationErrorType_c body_location;
  uint8_t has_body_face;
  pyramid_data_model_agra_BodyFaceType_c body_face;
} pyramid_data_model_agra_ImpactPointType_c;

typedef struct pyramid_data_model_agra_FuzeTriggerType_c {
  uint8_t has_fuze_distance;
  double fuze_distance;
  uint8_t has_fuze_delay_time;
  pyramid_str_t fuze_delay_time;
} pyramid_data_model_agra_FuzeTriggerType_c;

typedef struct pyramid_data_model_agra_FuzeType_c {
  pyramid_data_model_agra_FuzeTriggerType_c fuze_trigger;
  uint8_t has_fuze_mode;
  int32_t fuze_mode;
  uint8_t has_fuze_position;
  int32_t fuze_position;
  pyramid_str_t mnemonic;
  pyramid_str_t fuze_separation_arm_time;
} pyramid_data_model_agra_FuzeType_c;

typedef struct pyramid_data_model_agra_ImpactConditionsType_c {
  pyramid_slice_t fuze;
  uint8_t has_impact_speed;
  double impact_speed;
  uint8_t has_impact_point;
  pyramid_data_model_agra_ImpactPointType_c impact_point;
  uint8_t has_impact_angle;
  pyramid_data_model_agra_AzElReferenceType_c impact_angle;
  uint8_t has_spin_rate;
  uint32_t spin_rate;
} pyramid_data_model_agra_ImpactConditionsType_c;

typedef struct pyramid_data_model_agra_WeaponeeringType_c {
  uint8_t has_store_information;
  pyramid_data_model_agra_WeaponeeringStoreType_c store_information;
  uint8_t has_target_information;
  pyramid_data_model_agra_WeaponeeringTargetInfoType_c target_information;
  uint8_t has_approach_conditions;
  pyramid_data_model_agra_ApproachConditionsType_c approach_conditions;
  uint8_t has_impact_conditions;
  pyramid_data_model_agra_ImpactConditionsType_c impact_conditions;
} pyramid_data_model_agra_WeaponeeringType_c;

typedef struct pyramid_data_model_agra_StrikeTaskWeaponType_c {
  uint8_t has_store_information;
  pyramid_data_model_agra_WeaponeeringStoreType_c store_information;  /* from WeaponeeringType */
  uint8_t has_target_information;
  pyramid_data_model_agra_WeaponeeringTargetInfoType_c target_information;  /* from WeaponeeringType */
  uint8_t has_approach_conditions;
  pyramid_data_model_agra_ApproachConditionsType_c approach_conditions;  /* from WeaponeeringType */
  uint8_t has_impact_conditions;
  pyramid_data_model_agra_ImpactConditionsType_c impact_conditions;  /* from WeaponeeringType */
  uint8_t required_weapon;
} pyramid_data_model_agra_StrikeTaskWeaponType_c;

typedef struct pyramid_data_model_agra_StrikeTaskWeaponListType_c {
  pyramid_slice_t weapon;
} pyramid_data_model_agra_StrikeTaskWeaponListType_c;

typedef struct pyramid_data_model_agra_FlightTaskBaseType_c {
  pyramid_slice_t capability_type;
} pyramid_data_model_agra_FlightTaskBaseType_c;

typedef struct pyramid_data_model_agra_EA_TechniqueIdentifierType_c {
  pyramid_str_t key;  /* from ForeignKeyType */
  pyramid_str_t system_name;  /* from ForeignKeyType */
} pyramid_data_model_agra_EA_TechniqueIdentifierType_c;

typedef struct pyramid_data_model_agra_AltitudeRangePairType_c {
  double altitude;
  double range;
} pyramid_data_model_agra_AltitudeRangePairType_c;

typedef struct pyramid_data_model_agra_AirVolumeSensorReferencedType_c {
  double azimuth_scan_width;
  double azimuth_scan_center;
  double elevation_scan_width;
  double elevation_scan_center;
  double max_range_of_interest;
  double min_range_of_interest;
  int32_t azimuth_scan_stabilization;
  int32_t elevation_scan_stabilization;
  uint8_t has_elevation_scan_center_altitude_range_pair;
  pyramid_data_model_agra_AltitudeRangePairType_c elevation_scan_center_altitude_range_pair;
  uint8_t roll_stabilized;
} pyramid_data_model_agra_AirVolumeSensorReferencedType_c;

typedef struct pyramid_data_model_agra_EA_TargetPointingType_c {
  uint8_t has_location_data;
  pyramid_data_model_agra_TargetType_c location_data;
  uint8_t has_air_volume;
  pyramid_data_model_agra_AirVolumeSensorReferencedType_c air_volume;
} pyramid_data_model_agra_EA_TargetPointingType_c;

typedef struct pyramid_data_model_agra_EA_PowerType_c {
  uint8_t has_power_at_target;
  double power_at_target;
  uint8_t has_erp;
  double erp;
  uint8_t has_jto_s;
  double jto_s;
} pyramid_data_model_agra_EA_PowerType_c;

typedef struct pyramid_data_model_agra_EA_EmitterDataType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_emitter_type;
  pyramid_data_model_agra_EmitterIdentityType_c emitter_type;
  uint8_t has_signal_description;
  pyramid_data_model_agra_SignalSummaryType_c signal_description;
  uint8_t has_signal_id;
  pyramid_data_model_agra_SignalID_Type_c signal_id;
} pyramid_data_model_agra_EA_EmitterDataType_c;

typedef struct pyramid_data_model_agra_EA_TargetType_c {
  uint8_t has_emitter_data;
  pyramid_data_model_agra_EA_EmitterDataType_c emitter_data;
  uint8_t has_pointing;
  pyramid_data_model_agra_EA_TargetPointingType_c pointing;
  uint8_t has_power;
  pyramid_data_model_agra_EA_PowerType_c power;
  pyramid_str_t last_signal_timestamp;
} pyramid_data_model_agra_EA_TargetType_c;

typedef struct pyramid_data_model_agra_EA_TaskSuppressionConstraintsType_c {
  pyramid_data_model_agra_EA_TargetType_c ea_target;
  uint8_t has_priority;
  uint32_t priority;
  uint8_t has_technique_identifier;
  pyramid_data_model_agra_EA_TechniqueIdentifierType_c technique_identifier;
} pyramid_data_model_agra_EA_TaskSuppressionConstraintsType_c;

typedef struct pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c;

typedef struct pyramid_data_model_agra_EA_TaskThreatsType_c {
  uint8_t has_suppress_all;
  uint8_t suppress_all;
  uint8_t has_suppression_constraints;
  pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c suppression_constraints;
} pyramid_data_model_agra_EA_TaskThreatsType_c;

typedef struct pyramid_data_model_agra_ProtectedAssetAndThreatType_c {
  uint32_t priority;
  pyramid_data_model_agra_AssetType_c asset;
} pyramid_data_model_agra_ProtectedAssetAndThreatType_c;

typedef struct pyramid_data_model_agra_EA_TaskProtectedAssetsType_c {
  uint8_t protect_all;
  uint8_t self_protect;
  pyramid_slice_t protected_asset;
} pyramid_data_model_agra_EA_TaskProtectedAssetsType_c;

typedef struct pyramid_data_model_agra_ProtectedAssetType_c {
  uint32_t priority;
  pyramid_data_model_agra_AssetType_c asset;
} pyramid_data_model_agra_ProtectedAssetType_c;

typedef struct pyramid_data_model_agra_EA_TaskEscortType_c {
  pyramid_slice_t escorted_asset;
  uint8_t has_min_distance_offset;
  double min_distance_offset;
  uint8_t has_max_distance_offset;
  double max_distance_offset;
  uint8_t has_relative_angle_offset;
  pyramid_data_model_agra_AnglePairType_c relative_angle_offset;
  uint8_t has_rendezvous_point;
  pyramid_data_model_agra_Point4D_Type_c rendezvous_point;
} pyramid_data_model_agra_EA_TaskEscortType_c;

typedef struct pyramid_data_model_agra_EA_ResponseType_c {
  uint8_t has_route_requirements;
  pyramid_data_model_agra_EA_TaskEscortType_c route_requirements;
  uint8_t has_protected_assets;
  pyramid_data_model_agra_EA_TaskProtectedAssetsType_c protected_assets;
  uint8_t has_threats;
  pyramid_data_model_agra_EA_TaskThreatsType_c threats;
} pyramid_data_model_agra_EA_ResponseType_c;

typedef struct pyramid_data_model_agra_AMTI_CollectionConstraintsType_c {
  uint8_t has_map_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c map_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_grazing_angle_constraints;
  pyramid_data_model_agra_AngleQuarterPairType_c grazing_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_look_angle_constraints;
  pyramid_data_model_agra_AnglePairType_c look_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_elevation_angle_constraints;
  pyramid_data_model_agra_AngleHalfPairType_c elevation_angle_constraints;  /* from CollectionConstraintsType */
  uint8_t has_slant_range_constraints;
  pyramid_data_model_agra_SlantRangeConstraintsType_c slant_range_constraints;  /* from CollectionConstraintsType */
  uint8_t has_collection_pattern;
  int32_t collection_pattern;  /* from CollectionConstraintsType */
  uint8_t has_surface_mover_velocity_gate;
  double surface_mover_velocity_gate;
  uint8_t has_target_rcs;
  double target_rcs;
  uint8_t has_false_alarm_level;
  uint32_t false_alarm_level;
  uint8_t has_emcon;
  pyramid_data_model_agra_EmconConstraintType_c emcon;
  pyramid_slice_t target_radial_velocity;
} pyramid_data_model_agra_AMTI_CollectionConstraintsType_c;

typedef struct pyramid_data_model_agra_AMTI_TaskBaseType_c {
  int32_t capability_type;
  pyramid_slice_t sub_capability_type;
  uint8_t has_collection_policy;
  int32_t collection_policy;
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_AMTI_CollectionConstraintsType_c collection_constraints;
  pyramid_slice_t output;
} pyramid_data_model_agra_AMTI_TaskBaseType_c;

typedef struct pyramid_data_model_agra_TaskResponseType_c {
  uint8_t has_air_sample;
  pyramid_data_model_agra_AirSampleTaskBaseType_c air_sample;
  uint8_t has_amti;
  pyramid_data_model_agra_AMTI_TaskBaseType_c amti;
  uint8_t has_ao;
  pyramid_data_model_agra_AO_TaskBaseType_c ao;
  uint8_t has_cargo_delivery;
  pyramid_str_t cargo_delivery;
  uint8_t has_comint;
  pyramid_data_model_agra_COMINT_TaskBaseType_c comint;
  uint8_t has_comm_relay;
  pyramid_data_model_agra_CommRelayTaskBaseType_c comm_relay;
  uint8_t has_ea;
  pyramid_data_model_agra_EA_ResponseType_c ea;
  uint8_t has_esm;
  pyramid_data_model_agra_ESM_TaskBaseType_c esm;
  uint8_t has_flight;
  pyramid_data_model_agra_FlightTaskBaseType_c flight;
  uint8_t has_orbit_change;
  pyramid_data_model_agra_OrbitChangeTaskBaseType_c orbit_change;
  uint8_t has_orbital_surveillance;
  pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c orbital_surveillance;
  uint8_t has_po;
  pyramid_data_model_agra_PO_TaskBaseType_c po;
  uint8_t has_refuel;
  pyramid_data_model_agra_RefuelTaskBaseType_c refuel;
  uint8_t has_sar;
  pyramid_data_model_agra_SAR_TaskBaseType_c sar;
  uint8_t has_smti;
  pyramid_data_model_agra_SMTI_TaskBaseType_c smti;
  uint8_t has_strike;
  pyramid_data_model_agra_StrikeTaskWeaponListType_c strike;
  uint8_t has_system_deployment;
  pyramid_data_model_agra_SystemDeploymentTaskBaseType_c system_deployment;
  uint8_t has_tactical_order;
  pyramid_data_model_agra_TacticalOrderTaskBaseType_c tactical_order;
  uint8_t has_weather_radar;
  pyramid_str_t weather_radar;
} pyramid_data_model_agra_TaskResponseType_c;

typedef struct pyramid_data_model_agra_COMINT_CommandResponseType_c {
  int32_t sub_capability;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_COMINT_CommandResponseType_c;

typedef struct pyramid_data_model_agra_ESM_CommandResponseType_c {
  int32_t sub_capability;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_ESM_CommandResponseType_c;

typedef struct pyramid_data_model_agra_PO_ConstraintControlsType_c {
  uint8_t has_enable_disable;
  uint8_t enable_disable;
  uint8_t has_auto_enable_disable;
  uint8_t auto_enable_disable;
  pyramid_str_t reset;
} pyramid_data_model_agra_PO_ConstraintControlsType_c;

typedef struct pyramid_data_model_agra_PO_AngleConstraintControlsType_c {
  uint8_t has_controls;
  pyramid_data_model_agra_PO_ConstraintControlsType_c controls;
  uint8_t has_setting;
  pyramid_data_model_agra_AnglePairType_c setting;
} pyramid_data_model_agra_PO_AngleConstraintControlsType_c;

typedef struct pyramid_data_model_agra_GimbalAxisControlType_c {
  pyramid_data_model_agra_GimbalAxisID_Type_c axis_id;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c axis;
} pyramid_data_model_agra_GimbalAxisControlType_c;

typedef struct pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c {
  int32_t reference_frame;
  uint8_t has_component_id;
  pyramid_data_model_agra_ComponentID_Type_c component_id;
  uint8_t has_pitch_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c pitch_angle;
  uint8_t has_roll_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c roll_angle;
  uint8_t has_yaw_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c yaw_angle;
  pyramid_slice_t axis;
} pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c;

typedef struct pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c {
  uint8_t has_controls;
  pyramid_data_model_agra_PO_ConstraintControlsType_c controls;
  uint8_t has_setting;
  pyramid_data_model_agra_DistanceConstraintsType_c setting;
} pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c;

typedef struct pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c {
  uint8_t has_controls;
  pyramid_data_model_agra_PO_ConstraintControlsType_c controls;
  uint8_t has_setting;
  int32_t setting;
} pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c;

typedef struct pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c {
  uint8_t has_controls;
  pyramid_data_model_agra_PO_ConstraintControlsType_c controls;
  uint8_t has_setting;
  double setting;
} pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c;

typedef struct pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c {
  uint8_t has_map_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c map_angle;
  uint8_t has_grazing_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c grazing_angle;
  uint8_t has_look_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c look_angle;
  uint8_t has_elevation_angle;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c elevation_angle;
  uint8_t has_slant_range;
  pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c slant_range;
  uint8_t has_collection_pattern;
  pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c collection_pattern;
  uint8_t has_sun_elev_angles;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c sun_elev_angles;
  uint8_t has_sun_azimuth_angles;
  pyramid_data_model_agra_PO_AngleConstraintControlsType_c sun_azimuth_angles;
  uint8_t has_sweep_speed;
  pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c sweep_speed;
  uint8_t has_gimbal_orientation;
  pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c gimbal_orientation;
} pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c;

typedef struct pyramid_data_model_agra_PO_CommandResponseType_c {
  int32_t capability;
  pyramid_slice_t sensor_spectrum;
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c collection_constraints;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_PO_CommandResponseType_c;

typedef struct pyramid_data_model_agra_SMTI_CommandResponseType_c {
  int32_t capability;
  uint8_t has_sub_capability;
  int32_t sub_capability;
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_SMTI_CollectionConstraintsType_c collection_constraints;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_SMTI_CommandResponseType_c;

typedef struct pyramid_data_model_agra_CommRelayCommandResponseType_c {
  int32_t sub_capability;
} pyramid_data_model_agra_CommRelayCommandResponseType_c;

typedef struct pyramid_data_model_agra_AMTI_CommandResponseType_c {
  int32_t capability;
  uint8_t has_target_rcs;
  double target_rcs;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_AMTI_CommandResponseType_c;

typedef struct pyramid_data_model_agra_SAR_CommandResponseType_c {
  int32_t capability;
  int32_t sub_capability;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_SAR_CommandResponseType_c;

typedef struct pyramid_data_model_agra_EA_CommandResponseType_c {
  pyramid_slice_t protected_asset;
  uint8_t has_activation;
  int32_t activation;
} pyramid_data_model_agra_EA_CommandResponseType_c;

typedef struct pyramid_data_model_agra_StrikeWeaponCommandType_c {
  uint8_t has_select_for_key_load;
  uint8_t select_for_key_load;
  uint8_t has_assign_target;
  pyramid_data_model_agra_GeoLocatedObjectType_c assign_target;
  uint8_t has_weapon_arm;
  uint8_t weapon_arm;
  uint8_t has_select_for_jettison;
  uint8_t select_for_jettison;
  uint8_t has_generate_dynamic_lar;
  uint8_t generate_dynamic_lar;
  uint8_t has_select_for_release;
  uint8_t select_for_release;
  uint8_t has_select_ao_code;
  pyramid_data_model_agra_AO_CodeType_c select_ao_code;
} pyramid_data_model_agra_StrikeWeaponCommandType_c;

typedef struct pyramid_data_model_agra_AO_CommandResponseType_c {
  int32_t capability;
  uint8_t has_emission_constraints;
  pyramid_data_model_agra_OpticalCollectionConstraintsType_c emission_constraints;
} pyramid_data_model_agra_AO_CommandResponseType_c;

typedef struct pyramid_data_model_agra_AirSampleCommandResponseType_c {
  int32_t capability;
  pyramid_slice_t product_output;
} pyramid_data_model_agra_AirSampleCommandResponseType_c;

typedef struct pyramid_data_model_agra_CommandResponseType_c {
  uint8_t has_air_sample;
  pyramid_data_model_agra_AirSampleCommandResponseType_c air_sample;
  uint8_t has_amti;
  pyramid_data_model_agra_AMTI_CommandResponseType_c amti;
  uint8_t has_ao;
  pyramid_data_model_agra_AO_CommandResponseType_c ao;
  uint8_t has_comint;
  pyramid_data_model_agra_COMINT_CommandResponseType_c comint;
  uint8_t has_comm_relay;
  pyramid_data_model_agra_CommRelayCommandResponseType_c comm_relay;
  uint8_t has_ea;
  pyramid_data_model_agra_EA_CommandResponseType_c ea;
  uint8_t has_esm;
  pyramid_data_model_agra_ESM_CommandResponseType_c esm;
  uint8_t has_po;
  pyramid_data_model_agra_PO_CommandResponseType_c po;
  uint8_t has_sar;
  pyramid_data_model_agra_SAR_CommandResponseType_c sar;
  uint8_t has_smti;
  pyramid_data_model_agra_SMTI_CommandResponseType_c smti;
  uint8_t has_strike;
  pyramid_data_model_agra_StrikeWeaponCommandType_c strike;
  uint8_t has_weather_radar;
  pyramid_str_t weather_radar;
} pyramid_data_model_agra_CommandResponseType_c;

typedef struct pyramid_data_model_agra_ResponseOptionType_c {
  uint8_t has_effect;
  int32_t effect;
  uint8_t has_effect_id;
  pyramid_data_model_agra_EffectID_Type_c effect_id;
  uint8_t has_action;
  int32_t action;
  uint8_t has_action_id;
  pyramid_data_model_agra_ActionID_Type_c action_id;
  uint8_t has_task;
  pyramid_data_model_agra_TaskResponseType_c task;
  uint8_t has_task_id;
  pyramid_data_model_agra_TaskID_Type_c task_id;
  uint8_t has_capability_command;
  pyramid_data_model_agra_CommandResponseType_c capability_command;
  uint8_t has_capability_command_id;
  pyramid_data_model_agra_CommandID_Type_c capability_command_id;
} pyramid_data_model_agra_ResponseOptionType_c;

typedef struct pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c {
  uint8_t has_temporal_criticality;
  int32_t temporal_criticality;
  uint8_t has_start_time_window;
  pyramid_data_model_agra_DateTimeRangeType_c start_time_window;
  uint8_t has_window_on_first_only;
  uint8_t window_on_first_only;
  uint8_t has_end_time_window;
  pyramid_data_model_agra_DateTimeRangeType_c end_time_window;
} pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c;

typedef struct pyramid_data_model_agra_RequirementTemplateOptionType_c {
  uint32_t requirement_option_index;
  pyramid_data_model_agra_ResponseOptionType_c requirement;
  uint8_t has_requirement_constraints;
  pyramid_data_model_agra_RequirementConstraintsType_c requirement_constraints;
  uint8_t has_timing_constraints;
  pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c timing_constraints;
} pyramid_data_model_agra_RequirementTemplateOptionType_c;

typedef struct pyramid_data_model_agra_RequirementGenerationDependencyType_c {
  int32_t dependency_type;  /* from RequirementDependencyBaseType */
  int32_t dependency_extent;  /* from RequirementDependencyBaseType */
  pyramid_str_t earliest_time;  /* from RequirementDependencyBaseType */
  pyramid_str_t latest_time;  /* from RequirementDependencyBaseType */
  uint32_t reference_requirement_index;
} pyramid_data_model_agra_RequirementGenerationDependencyType_c;

typedef struct pyramid_data_model_agra_RequirementTemplateOptionsType_c {
  uint32_t requirement_index;
  pyramid_str_t descriptive_label;
  pyramid_slice_t requirement_option;
  pyramid_slice_t dependent_requirement;
  pyramid_slice_t collection_objective;
  pyramid_str_t product_staleness;
} pyramid_data_model_agra_RequirementTemplateOptionsType_c;

typedef struct pyramid_data_model_agra_RequirementsTemplateType_c {
  pyramid_data_model_agra_RequirementsTemplateID_Type_c requirement_template_id;
  pyramid_slice_t requirement;
} pyramid_data_model_agra_RequirementsTemplateType_c;

typedef struct pyramid_data_model_agra_MA_ResponseMDT_c {
  pyramid_data_model_agra_ResponseID_Type_c response_id;
  pyramid_slice_t response_type;
  uint8_t has_response_management_constraints;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c response_management_constraints;
  uint8_t has_response_management_guidance;
  pyramid_data_model_agra_RequirementGuidanceType_c response_management_guidance;
  pyramid_slice_t option;
  pyramid_slice_t requirements_template;
  uint8_t has_metadata;
  pyramid_data_model_agra_RequirementMetadataType_c metadata;
} pyramid_data_model_agra_MA_ResponseMDT_c;

typedef struct pyramid_data_model_agra_MA_ResponseMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MA_ResponseMDT_c message_data;
} pyramid_data_model_agra_MA_ResponseMT_c;

typedef struct pyramid_data_model_agra_OperatorNameType_c {
  pyramid_str_t first_name;
  pyramid_str_t last_name;
} pyramid_data_model_agra_OperatorNameType_c;

typedef struct pyramid_data_model_agra_ContactDetailsType_c {
  pyramid_data_model_agra_OperatorNameType_c operator_name;
  pyramid_slice_t phone_number;
} pyramid_data_model_agra_ContactDetailsType_c;

typedef struct pyramid_data_model_agra_UnitID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_UnitID_Type_c;

typedef struct pyramid_data_model_agra_UnitIdentityType_c {
  pyramid_data_model_agra_UnitID_Type_c unit_uuid;
  pyramid_str_t identifier;
  pyramid_str_t name;
  pyramid_str_t surrogate_key;
  pyramid_slice_t ob_type;
  pyramid_str_t symbol_code;
} pyramid_data_model_agra_UnitIdentityType_c;

typedef struct pyramid_data_model_agra_AppliesToType_c {
  pyramid_slice_t system_id;
  pyramid_slice_t subsystem_id;
  pyramid_slice_t service_id;
} pyramid_data_model_agra_AppliesToType_c;

typedef struct pyramid_data_model_agra_ReportToType_c {
  uint8_t has_applies_to;
  pyramid_data_model_agra_AppliesToType_c applies_to;
  uint8_t has_contact_details;
  pyramid_data_model_agra_ContactDetailsType_c contact_details;
  uint8_t has_unit;
  pyramid_data_model_agra_UnitIdentityType_c unit;
} pyramid_data_model_agra_ReportToType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorReportingCategoriesType_c {
  pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c sda_special_instructions;
  pyramid_data_model_agra_ComparableRankingType_c relative_priority;
  uint8_t has_product_needed_by;
  pyramid_data_model_agra_ProductNeededByType_c product_needed_by;
  pyramid_slice_t report_to;
} pyramid_data_model_agra_OrbitalSurveillanceSensorReportingCategoriesType_c;

typedef struct pyramid_data_model_agra_PercentileRCSType_c {
  double rcs;
  uint8_t has_percentile;
  double percentile;
} pyramid_data_model_agra_PercentileRCSType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c {
  uint8_t has_radar_cross_section;
  pyramid_data_model_agra_PercentileRCSType_c radar_cross_section;
  uint8_t has_visual_magnitude;
  double visual_magnitude;
  uint8_t has_area;
  double area;
  uint8_t has_intensity;
  double intensity;
} pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c {
  uint8_t has_minimum_size;
  pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c minimum_size;
  uint8_t has_minimum_signal_strength;
  double minimum_signal_strength;
} pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c;

typedef struct pyramid_data_model_agra_ObservationsPerTrackLimitsType_c {
  int32_t max;
  uint8_t has_min;
  int32_t min;
} pyramid_data_model_agra_ObservationsPerTrackLimitsType_c;

typedef struct pyramid_data_model_agra_MetricCollectionType_c {
  uint8_t use_all_passes;
  uint8_t has_number_of_observations;
  pyramid_data_model_agra_ObservationsPerTrackLimitsType_c number_of_observations;
  uint8_t has_centered_at_max_predicted_snr;
  uint8_t centered_at_max_predicted_snr;
  uint8_t has_spacing_of_observations;
  int32_t spacing_of_observations;
  uint8_t has_report_point_soi;
  uint8_t report_point_soi;
  uint8_t has_quality_of_obs;
  int32_t quality_of_obs;
} pyramid_data_model_agra_MetricCollectionType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c {
  uint8_t has_target_rotational_periods;
  int32_t target_rotational_periods;
  uint8_t has_time;
  pyramid_str_t time;
} pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c;

typedef struct pyramid_data_model_agra_SizeEstimationCharacterizationType_c {
  uint8_t has_min_collection;
  pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c min_collection;
  uint8_t has_size_data;
  int32_t size_data;
} pyramid_data_model_agra_SizeEstimationCharacterizationType_c;

typedef struct pyramid_data_model_agra_StructureAssessmentCharacterizationType_c {
  pyramid_data_model_agra_SizeEstimationCharacterizationType_c size_estimation;
  pyramid_data_model_agra_ResolvedCharacterizationType_c resolved;
} pyramid_data_model_agra_StructureAssessmentCharacterizationType_c;

typedef struct pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c {
  int32_t all;
  int32_t any;
} pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c;

typedef struct pyramid_data_model_agra_SensorCharacterizationChoiceType_c {
  uint8_t has_phemonemology_specific;
  pyramid_data_model_agra_CharacterizationChoiceType_c phemonemology_specific;
  uint8_t has_stability_and_orientation_assessment;
  pyramid_data_model_agra_StabilityCharacterizationType_c stability_and_orientation_assessment;
  uint8_t has_structure_assessment;
  pyramid_data_model_agra_StructureAssessmentCharacterizationType_c structure_assessment;
  uint8_t has_identification_verification;
  pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c identification_verification;
  uint8_t has_operations_changes;
  pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c operations_changes;
} pyramid_data_model_agra_SensorCharacterizationChoiceType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceCollectionRequirementsType_c {
  uint8_t has_metric_collection;
  pyramid_data_model_agra_MetricCollectionType_c metric_collection;
  uint8_t has_search;
  pyramid_data_model_agra_SpeedRangeType_c search;
  uint8_t has_orbit_determination;
  pyramid_data_model_agra_OrbitAccuracyType_c orbit_determination;
  uint8_t has_characterization;
  pyramid_data_model_agra_SensorCharacterizationChoiceType_c characterization;
  uint8_t has_multi_object;
  pyramid_data_model_agra_MultiObjectType_c multi_object;
  uint8_t has_maneuver_detection;
  pyramid_data_model_agra_ManeuverDetectionType_c maneuver_detection;
  uint8_t has_deployment_detection;
  pyramid_data_model_agra_DeploymentDetectionType_c deployment_detection;
} pyramid_data_model_agra_OrbitalSurveillanceCollectionRequirementsType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorTaskBaseType_c {
  int32_t capability_type;
  pyramid_slice_t collection_requirements;
  uint8_t confirm_object_acquisition;
  pyramid_slice_t information_objective;
  pyramid_slice_t reporting_requirements;
  pyramid_str_t set_up_start_time;
  pyramid_str_t post_collection_wrap_up;
  uint8_t has_sensitivity_constraints;
  pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c sensitivity_constraints;
} pyramid_data_model_agra_OrbitalSurveillanceSensorTaskBaseType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c {
  pyramid_data_model_agra_OrbitKinematicsType_c orbital_kinematics;
  uint8_t has_relative_search_volume;
  pyramid_data_model_agra_Shape3D_ChoiceType_c relative_search_volume;
} pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c;

typedef struct pyramid_data_model_agra_TLE_Type_c {
  pyramid_data_model_agra_COE_OrbitBaseType_c base;  /* from TLE_BaseType */
  pyramid_str_t epoch;  /* from TLE_BaseType */
  uint8_t has_element_set_number;
  uint32_t element_set_number;  /* from TLE_BaseType */
  uint8_t has_first_time_derivative_of_mean_motion;
  double first_time_derivative_of_mean_motion;  /* from TLE_BaseType */
  uint8_t has_second_time_derivative_of_mean_motion;
  double second_time_derivative_of_mean_motion;  /* from TLE_BaseType */
  uint8_t has_bstar_drag;
  double bstar_drag;  /* from TLE_BaseType */
  pyramid_data_model_agra_COE_PositionType_c position;  /* from TLE_BaseType */
  double mean_motion;  /* from TLE_BaseType */
  uint8_t has_revolution_number;
  uint32_t revolution_number;  /* from TLE_BaseType */
  pyramid_data_model_agra_SatelliteIdentifierType_c satellite_identity;
  int32_t classification;
  uint32_t ephemeris_type;
} pyramid_data_model_agra_TLE_Type_c;

typedef struct pyramid_data_model_agra_ElementSetCloudType_c {
  pyramid_slice_t element_set;
} pyramid_data_model_agra_ElementSetCloudType_c;

typedef struct pyramid_data_model_agra_SatelliteIdentityType_c {
  uint8_t has_standard;
  pyramid_data_model_agra_StandardIdentityType_c standard;
  uint8_t has_satellite;
  pyramid_data_model_agra_SatelliteIdentifierType_c satellite;
} pyramid_data_model_agra_SatelliteIdentityType_c;

typedef struct pyramid_data_model_agra_SatelliteIdentityChoiceType_c {
  uint8_t has_by_instance;
  pyramid_data_model_agra_AssetType_c by_instance;
  uint8_t has_by_type;
  pyramid_data_model_agra_SatelliteIdentityType_c by_type;
} pyramid_data_model_agra_SatelliteIdentityChoiceType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceObjectBaseType_c {
  pyramid_data_model_agra_SatelliteIdentityChoiceType_c satellite_identity;
  uint8_t has_kinematics_source;
  int32_t kinematics_source;
  uint8_t has_orbit_kinematics_override;
  pyramid_data_model_agra_OrbitKinematicsType_c orbit_kinematics_override;
} pyramid_data_model_agra_OrbitalSurveillanceObjectBaseType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceObjectType_c {
  pyramid_data_model_agra_SatelliteIdentityChoiceType_c satellite_identity;  /* from OrbitalSurveillanceObjectBaseType */
  uint8_t has_kinematics_source;
  int32_t kinematics_source;  /* from OrbitalSurveillanceObjectBaseType */
  uint8_t has_orbit_kinematics_override;
  pyramid_data_model_agra_OrbitKinematicsType_c orbit_kinematics_override;  /* from OrbitalSurveillanceObjectBaseType */
  uint8_t has_relative_search_volume;
  pyramid_data_model_agra_Shape3D_ChoiceType_c relative_search_volume;
} pyramid_data_model_agra_OrbitalSurveillanceObjectType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c {
  pyramid_data_model_agra_OrbitalSurveillanceObjectType_c primary_object;
  pyramid_slice_t secondary_objects;
} pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c;

typedef struct pyramid_data_model_agra_AngleRateRangeType_c {
  double min;
  double max;
} pyramid_data_model_agra_AngleRateRangeType_c;

typedef struct pyramid_data_model_agra_DoubleMinMaxType_c {
  double min;
  double max;
} pyramid_data_model_agra_DoubleMinMaxType_c;

typedef struct pyramid_data_model_agra_CapabilityCoverageAreaID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_CapabilityCoverageAreaID_Type_c;

typedef struct pyramid_data_model_agra_SourceCoverageType_c {
  uint8_t has_fov_reference_frame;
  int32_t fov_reference_frame;
  uint8_t has_range_limits;
  pyramid_data_model_agra_SlantRangeConstraintsType_c range_limits;
  uint8_t has_range_rate_limits;
  pyramid_data_model_agra_SpeedRangeType_c range_rate_limits;
  uint8_t has_azimuth_limits;
  pyramid_data_model_agra_AnglePairType_c azimuth_limits;
  uint8_t has_elevation_limits;
  pyramid_data_model_agra_AngleHalfPairType_c elevation_limits;
  uint8_t has_azimuth_angle_rate_limits;
  pyramid_data_model_agra_AngleRateRangeType_c azimuth_angle_rate_limits;
  uint8_t has_elevation_angle_rate_limits;
  pyramid_data_model_agra_AngleRateRangeType_c elevation_angle_rate_limits;
  uint8_t has_coscone_y_range_limits;
  pyramid_data_model_agra_DoubleMinMaxType_c coscone_y_range_limits;
  uint8_t has_coscone_z_range_limits;
  pyramid_data_model_agra_DoubleMinMaxType_c coscone_z_range_limits;
  uint8_t has_coscone_y_rate_range_limits;
  pyramid_data_model_agra_DoubleMinMaxType_c coscone_y_rate_range_limits;
  uint8_t has_coscone_z_rate_range_limits;
  pyramid_data_model_agra_DoubleMinMaxType_c coscone_z_rate_range_limits;
  pyramid_slice_t capability_coverage_area_id;
} pyramid_data_model_agra_SourceCoverageType_c;

typedef struct pyramid_data_model_agra_AzimuthElevationRangePointType_c {
  pyramid_str_t time;
  double azimuth;
  double elevation;
  double range;
} pyramid_data_model_agra_AzimuthElevationRangePointType_c;

typedef struct pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c;

typedef struct pyramid_data_model_agra_RightAscensionDeclinationPointType_c {
  pyramid_str_t time;
  double right_ascension;
  double declination;
} pyramid_data_model_agra_RightAscensionDeclinationPointType_c;

typedef struct pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c;

typedef struct pyramid_data_model_agra_SensorPointListType_Point3DList_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_SensorPointListType_Point3DList_List_c;

typedef struct pyramid_data_model_agra_SensorPointListType_c {
  uint8_t has_azimuth_elevation_range_point_list;
  pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c azimuth_elevation_range_point_list;
  uint8_t has_right_ascension_declination_point_list;
  pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c right_ascension_declination_point_list;
  uint8_t has_point3_d_list;
  pyramid_data_model_agra_SensorPointListType_Point3DList_List_c point3_d_list;
} pyramid_data_model_agra_SensorPointListType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c {
  uint8_t has_point_list;
  pyramid_data_model_agra_SensorPointListType_c point_list;
  uint8_t has_element_set_cloud;
  pyramid_data_model_agra_ElementSetCloudType_c element_set_cloud;
  uint8_t has_object_based;
  pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c object_based;
  uint8_t has_location_based;
  pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c location_based;
  uint8_t has_sensor_centric_volume;
  pyramid_data_model_agra_SourceCoverageType_c sensor_centric_volume;
} pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c {
  int32_t capability_type;  /* from OrbitalSurveillanceSensorTaskBaseType */
  pyramid_slice_t collection_requirements;  /* from OrbitalSurveillanceSensorTaskBaseType */
  uint8_t confirm_object_acquisition;  /* from OrbitalSurveillanceSensorTaskBaseType */
  pyramid_slice_t information_objective;  /* from OrbitalSurveillanceSensorTaskBaseType */
  pyramid_slice_t reporting_requirements;  /* from OrbitalSurveillanceSensorTaskBaseType */
  pyramid_str_t set_up_start_time;  /* from OrbitalSurveillanceSensorTaskBaseType */
  pyramid_str_t post_collection_wrap_up;  /* from OrbitalSurveillanceSensorTaskBaseType */
  uint8_t has_sensitivity_constraints;
  pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c sensitivity_constraints;  /* from OrbitalSurveillanceSensorTaskBaseType */
  pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c target;
} pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c;

typedef struct pyramid_data_model_agra_AltitudeConstraintsType_c {
  uint8_t has_min;
  double min;
  uint8_t has_max;
  double max;
} pyramid_data_model_agra_AltitudeConstraintsType_c;

typedef struct pyramid_data_model_agra_AreaConstraintsType_c {
  uint8_t has_distance_constraints;
  pyramid_data_model_agra_DistanceConstraintsType_c distance_constraints;
  uint8_t has_altitude_constraints;
  pyramid_data_model_agra_AltitudeConstraintsType_c altitude_constraints;
} pyramid_data_model_agra_AreaConstraintsType_c;

typedef struct pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c {
  uint8_t has_release_point;
  pyramid_data_model_agra_Point3D_Type_c release_point;
  uint8_t has_release_area;
  pyramid_data_model_agra_AreaConstraintsType_c release_area;
} pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c;

typedef struct pyramid_data_model_agra_TargetInformationType_c {
  uint8_t has_number_of_dmp_is;
  uint32_t number_of_dmp_is;
  uint8_t has_target_type;
  int32_t target_type;
  uint8_t has_target_defenses;
  int32_t target_defenses;
} pyramid_data_model_agra_TargetInformationType_c;

typedef struct pyramid_data_model_agra_StrikeTaskType_c {
  pyramid_data_model_agra_TargetType_c target;
  uint8_t has_target_information;
  pyramid_data_model_agra_TargetInformationType_c target_information;
  uint8_t has_ingress_constraint;
  pyramid_data_model_agra_AnglePairType_c ingress_constraint;
  uint8_t has_egress_constraint;
  pyramid_data_model_agra_AnglePairType_c egress_constraint;
  uint8_t has_initial_point;
  pyramid_data_model_agra_Point3D_Type_c initial_point;
  uint8_t has_release_constraints;
  pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c release_constraints;
  uint8_t has_weapon_list;
  pyramid_data_model_agra_StrikeTaskWeaponListType_c weapon_list;
  pyramid_str_t no_strike;
} pyramid_data_model_agra_StrikeTaskType_c;

typedef struct pyramid_data_model_agra_MA_EscortAssetType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_system_id;
  pyramid_data_model_agra_SystemID_Type_c system_id;
} pyramid_data_model_agra_MA_EscortAssetType_c;

typedef struct pyramid_data_model_agra_MA_EscortFollowPathType_c {
  double separation;
  uint8_t has_route_plan_id;
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
} pyramid_data_model_agra_MA_EscortFollowPathType_c;

typedef struct pyramid_data_model_agra_MA_EscortReferenceType_c {
  uint8_t has_follow_path;
  pyramid_data_model_agra_MA_EscortFollowPathType_c follow_path;
  uint8_t has_geographic_reference;
  pyramid_data_model_agra_LOS_Type_c geographic_reference;
  uint8_t has_body_reference;
  pyramid_data_model_agra_LOS_Type_c body_reference;
} pyramid_data_model_agra_MA_EscortReferenceType_c;

typedef struct pyramid_data_model_agra_MA_EscortTaskType_c {
  pyramid_data_model_agra_MA_EscortAssetType_c escort_asset;
  pyramid_data_model_agra_MA_EscortReferenceType_c reference;
} pyramid_data_model_agra_MA_EscortTaskType_c;

typedef struct pyramid_data_model_agra_TaskPlanConstraintType_c {
  uint8_t has_allocation_plan_id;
  pyramid_data_model_agra_TaskPlanID_Type_c allocation_plan_id;
  int32_t changeable_allocations;
  pyramid_slice_t task_type;
} pyramid_data_model_agra_TaskPlanConstraintType_c;

typedef struct pyramid_data_model_agra_ConstrainingPlansType_c {
  pyramid_slice_t constraining_plan_type;
  pyramid_slice_t mission_plan_id;
  pyramid_slice_t task_plan;
  pyramid_slice_t route_plan;
  pyramid_slice_t route_activity_plan;
  pyramid_slice_t orbit_plan;
  pyramid_slice_t orbit_activity_plan;
  pyramid_slice_t activity_plan;
  pyramid_slice_t comm_allocation;
  pyramid_slice_t effect_plan;
  pyramid_slice_t action_plan;
  pyramid_slice_t response_plan;
} pyramid_data_model_agra_ConstrainingPlansType_c;

typedef struct pyramid_data_model_agra_OtherSystemConstrainingPlansType_c {
  pyramid_data_model_agra_SystemID_Type_c constraining_system_id;
  int32_t constraint_usage;
  pyramid_data_model_agra_ConstrainingPlansType_c contraining_plans;
} pyramid_data_model_agra_OtherSystemConstrainingPlansType_c;

typedef struct pyramid_data_model_agra_PlanningCandidateBaseType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_ConstrainingPlansType_c constraining_plans;
  pyramid_slice_t other_system_constraining_plans;
} pyramid_data_model_agra_PlanningCandidateBaseType_c;

typedef struct pyramid_data_model_agra_RequirementPlanningCandidateType_c {
  pyramid_data_model_agra_SystemID_Type_c system_id;  /* from PlanningCandidateBaseType */
  uint8_t has_constraining_plans;
  pyramid_data_model_agra_ConstrainingPlansType_c constraining_plans;  /* from PlanningCandidateBaseType */
  pyramid_slice_t other_system_constraining_plans;  /* from PlanningCandidateBaseType */
  uint8_t has_route_guideline;
  pyramid_data_model_agra_PlanningGuidelineType_c route_guideline;
  uint8_t has_orbit_guideline;
  pyramid_data_model_agra_OrbitGuidelineType_c orbit_guideline;
} pyramid_data_model_agra_RequirementPlanningCandidateType_c;

typedef struct pyramid_data_model_agra_IdentityKindAssetType_c {
  uint8_t has_by_instance;
  pyramid_data_model_agra_AssetType_c by_instance;
  uint8_t has_by_identity;
  pyramid_data_model_agra_IdentityType_c by_identity;
  uint8_t has_by_plan;
  pyramid_data_model_agra_RequirementPlanningCandidateType_c by_plan;
} pyramid_data_model_agra_IdentityKindAssetType_c;

typedef struct pyramid_data_model_agra_RefuelTaskType_c {
  int32_t capability_type;  /* from RefuelTaskBaseType */
  pyramid_slice_t taker;
} pyramid_data_model_agra_RefuelTaskType_c;

typedef struct pyramid_data_model_agra_SMTI_TaskType_c {
  int32_t capability_type;  /* from SMTI_TaskBaseType */
  uint8_t has_sub_capability_type;
  int32_t sub_capability_type;  /* from SMTI_TaskBaseType */
  uint8_t has_look_at_coords;
  pyramid_data_model_agra_LocatedEllipseType_c look_at_coords;  /* from SMTI_TaskBaseType */
  uint8_t has_collection_options;
  pyramid_data_model_agra_SMTI_CollectionOptionsType_c collection_options;  /* from SMTI_TaskBaseType */
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_SMTI_CollectionConstraintsType_c collection_constraints;  /* from SMTI_TaskBaseType */
  pyramid_slice_t output;  /* from SMTI_TaskBaseType */
  pyramid_data_model_agra_TargetType_c target;
} pyramid_data_model_agra_SMTI_TaskType_c;

typedef struct pyramid_data_model_agra_WeatherRadarTaskType_c {
  int32_t capability_type;
  pyramid_data_model_agra_ZoneType_c air_volume_location;
  uint8_t has_collection_policy;
  int32_t collection_policy;
  uint8_t has_repetition;
  pyramid_data_model_agra_RepetitionType_c repetition;
  pyramid_slice_t output;
} pyramid_data_model_agra_WeatherRadarTaskType_c;

typedef struct pyramid_data_model_agra_ZoneChoiceType_c {
  uint8_t has_op_zone_id;
  pyramid_data_model_agra_OpZoneID_Type_c op_zone_id;
  uint8_t has_zone_target;
  pyramid_data_model_agra_ZoneExternalType_c zone_target;
} pyramid_data_model_agra_ZoneChoiceType_c;

typedef struct pyramid_data_model_agra_OrbitDurationType_c {
  uint8_t has_time;
  pyramid_str_t time;
  uint8_t has_number_of_orbits;
  uint32_t number_of_orbits;
} pyramid_data_model_agra_OrbitDurationType_c;

typedef struct pyramid_data_model_agra_CircleType_c {
  pyramid_data_model_agra_PointChoiceType_c point_choice;
  double radius;
} pyramid_data_model_agra_CircleType_c;

typedef struct pyramid_data_model_agra_OrbitType_c {
  int32_t orbit_type;
  pyramid_slice_t circle;
  int32_t turn_direction;
  pyramid_data_model_agra_OrbitDurationType_c duration;
} pyramid_data_model_agra_OrbitType_c;

typedef struct pyramid_data_model_agra_Point3D_RelativeType_c {
  pyramid_data_model_agra_ReferenceFrameID_Type_c reference_frame_id;
  uint8_t has_relative_offset;
  pyramid_data_model_agra_RelativeOffset3D_Type_c relative_offset;
} pyramid_data_model_agra_Point3D_RelativeType_c;

typedef struct pyramid_data_model_agra_PointChoice3D_Type_c {
  uint8_t has_absolute_point;
  pyramid_data_model_agra_Point3D_Type_c absolute_point;
  uint8_t has_relative_point;
  pyramid_data_model_agra_Point3D_RelativeType_c relative_point;
} pyramid_data_model_agra_PointChoice3D_Type_c;

typedef struct pyramid_data_model_agra_HoverType_c {
  pyramid_data_model_agra_PointChoice3D_Type_c point3_d;
  pyramid_str_t duration;
} pyramid_data_model_agra_HoverType_c;

typedef struct pyramid_data_model_agra_LoiterType_c {
  uint8_t has_orbit;
  pyramid_data_model_agra_OrbitType_c orbit;
  uint8_t has_hover;
  pyramid_data_model_agra_HoverType_c hover;
} pyramid_data_model_agra_LoiterType_c;

typedef struct pyramid_data_model_agra_VolumeChoiceType_c {
  uint8_t has_op_volume_id;
  pyramid_data_model_agra_OpVolumeID_Type_c op_volume_id;
  uint8_t has_volume_target;
  pyramid_data_model_agra_OpVolumeType_c volume_target;
} pyramid_data_model_agra_VolumeChoiceType_c;

typedef struct pyramid_data_model_agra_PathType_c {
  pyramid_slice_t segment_vertex;
  int32_t line_projection;
} pyramid_data_model_agra_PathType_c;

typedef struct pyramid_data_model_agra_EA_TaskRouteRequirementsType_c {
  uint8_t has_path;
  pyramid_data_model_agra_PathType_c path;
  uint8_t has_loiter;
  pyramid_data_model_agra_LoiterType_c loiter;
  uint8_t has_escort;
  pyramid_data_model_agra_EA_TaskEscortType_c escort;
  uint8_t has_zone_constraints;
  pyramid_data_model_agra_ZoneChoiceType_c zone_constraints;
  uint8_t has_volume_constraints;
  pyramid_data_model_agra_VolumeChoiceType_c volume_constraints;
} pyramid_data_model_agra_EA_TaskRouteRequirementsType_c;

typedef struct pyramid_data_model_agra_EA_TaskType_c {
  uint8_t has_route_requirements;
  pyramid_data_model_agra_EA_TaskRouteRequirementsType_c route_requirements;
  pyramid_data_model_agra_EA_TaskProtectedAssetsType_c protected_assets;
  pyramid_data_model_agra_EA_TaskThreatsType_c threats;
} pyramid_data_model_agra_EA_TaskType_c;

typedef struct pyramid_data_model_agra_DeployableSystemIdentityType_c {
  uint8_t has_platform;
  pyramid_data_model_agra_PlatformIdentityType_c platform;
  uint8_t has_specific;
  pyramid_data_model_agra_SpecificIdentityType_c specific;
  uint8_t has_specific_vehicle;
  pyramid_data_model_agra_VehicleIdentificationType_c specific_vehicle;
  uint8_t has_weapon;
  pyramid_data_model_agra_StoreType_c weapon;
} pyramid_data_model_agra_DeployableSystemIdentityType_c;

typedef struct pyramid_data_model_agra_SystemDeploymentTaskType_c {
  int32_t capability_type;  /* from SystemDeploymentTaskBaseType */
  pyramid_data_model_agra_DeployableSystemIdentityType_c system_to_deploy;
  pyramid_data_model_agra_KinematicsMultiStandardType_c deployment_kinematics;
} pyramid_data_model_agra_SystemDeploymentTaskType_c;

typedef struct pyramid_data_model_agra_SupportCapabilityID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_SupportCapabilityID_Type_c;

typedef struct pyramid_data_model_agra_AntennaResourceID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_AntennaResourceID_Type_c;

typedef struct pyramid_data_model_agra_AntennaResourceChoiceType_c {
  uint8_t has_antenna_resource_type_id;
  pyramid_data_model_agra_AntennaResourceID_Type_c antenna_resource_type_id;
  uint8_t has_antenna_resource_instance_id;
  pyramid_data_model_agra_AntennaResourceID_Type_c antenna_resource_instance_id;
} pyramid_data_model_agra_AntennaResourceChoiceType_c;

typedef struct pyramid_data_model_agra_SelectAntennaType_c {
  uint8_t has_support_capability_id;
  pyramid_data_model_agra_SupportCapabilityID_Type_c support_capability_id;
  uint8_t has_antenna_resource_choice;
  pyramid_data_model_agra_AntennaResourceChoiceType_c antenna_resource_choice;
} pyramid_data_model_agra_SelectAntennaType_c;

typedef struct pyramid_data_model_agra_ESM_TargetType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_emitter_type;
  pyramid_data_model_agra_EmitterIdentityType_c emitter_type;
  uint8_t has_specific_emitter;
  pyramid_data_model_agra_SpecificEmitterIdentityType_c specific_emitter;
  uint8_t has_signal_description;
  pyramid_data_model_agra_SignalSummaryType_c signal_description;
  uint8_t has_signal_id;
  pyramid_data_model_agra_SignalID_Type_c signal_id;
  uint8_t has_emitter_priority_bin;
  uint32_t emitter_priority_bin;
} pyramid_data_model_agra_ESM_TargetType_c;

typedef struct pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c {
  int32_t geolocation_state;
  pyramid_slice_t geolocation_type;
} pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c;

typedef struct pyramid_data_model_agra_NED_LOS_Type_c {
  double bearing;
  double elevation;
  uint8_t has_slant_range;
  double slant_range;
} pyramid_data_model_agra_NED_LOS_Type_c;

typedef struct pyramid_data_model_agra_NED_ConeType_c {
  pyramid_data_model_agra_NED_LOS_Type_c ned_los;
  double azimuth_extents;
  double elevation_extents;
} pyramid_data_model_agra_NED_ConeType_c;

typedef struct pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_dwell_fov;
  pyramid_data_model_agra_NED_ConeType_c dwell_fov;
  uint8_t has_point_target;
  pyramid_data_model_agra_PointTargetType_c point_target;
} pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c;

typedef struct pyramid_data_model_agra_ESM_LocationType_c {
  uint8_t has_target_location_data;
  pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c target_location_data;
  uint8_t has_esm_air_volume;
  pyramid_data_model_agra_AirVolumeSensorReferencedType_c esm_air_volume;
} pyramid_data_model_agra_ESM_LocationType_c;

typedef struct pyramid_data_model_agra_CollectionFrequencyType_c {
  double tune_frequency;
  uint8_t has_range;
  pyramid_data_model_agra_FrequencyRangeType_c range;
} pyramid_data_model_agra_CollectionFrequencyType_c;

typedef struct pyramid_data_model_agra_PulseDataCollectCommandType_c {
  pyramid_data_model_agra_CollectionFrequencyType_c collection_frequency;
  pyramid_str_t start_time;
  pyramid_str_t dwell_duration;
  uint8_t has_dwells;
  uint32_t dwells;
  pyramid_str_t repetition_interval;
  pyramid_str_t pulse_width;
  pyramid_str_t pri;
  uint8_t has_signal_bandwidth;
  pyramid_data_model_agra_FrequencyRangeType_c signal_bandwidth;
  uint8_t has_modulation;
  int32_t modulation;
  uint8_t has_data_collect_type;
  int32_t data_collect_type;
  uint8_t has_sample_rate;
  float sample_rate;
  uint8_t has_fft_points_per_sample;
  int32_t fft_points_per_sample;
} pyramid_data_model_agra_PulseDataCollectCommandType_c;

typedef struct pyramid_data_model_agra_SubCapabilityDetailsType_c {
  uint8_t has_select_antenna;
  pyramid_data_model_agra_SelectAntennaType_c select_antenna;
  pyramid_slice_t frequency_restriction;
  pyramid_slice_t target_emitter_data;
  uint8_t has_esm_location;
  pyramid_data_model_agra_ESM_LocationType_c esm_location;
  uint8_t has_pulse_data_collection;
  pyramid_data_model_agra_PulseDataCollectCommandType_c pulse_data_collection;
  uint8_t has_geolocation;
  pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c geolocation;
} pyramid_data_model_agra_SubCapabilityDetailsType_c;

typedef struct pyramid_data_model_agra_ESM_TaskType_c {
  pyramid_slice_t sub_capability_type;  /* from ESM_TaskBaseType */
  pyramid_slice_t output;  /* from ESM_TaskBaseType */
  pyramid_data_model_agra_SubCapabilityDetailsType_c sub_capability_details;
} pyramid_data_model_agra_ESM_TaskType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_dwell_fov;
  pyramid_data_model_agra_NED_ConeType_c dwell_fov;
  uint8_t has_point_target;
  pyramid_data_model_agra_PointTargetType_c point_target;
} pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c;

typedef struct pyramid_data_model_agra_COMINT_TargetType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_emitter_type;
  pyramid_data_model_agra_EmitterIdentityType_c emitter_type;
  uint8_t has_specific_emitter;
  pyramid_data_model_agra_SpecificEmitterIdentityType_c specific_emitter;
  uint8_t has_signal_description;
  pyramid_data_model_agra_SignalSummaryType_c signal_description;
  uint8_t has_signal_id;
  pyramid_data_model_agra_SignalID_Type_c signal_id;
  uint8_t has_target_class;
  pyramid_data_model_agra_ForeignKeyType_c target_class;
} pyramid_data_model_agra_COMINT_TargetType_c;

typedef struct pyramid_data_model_agra_COMINT_AcquisitionTargetType_c {
  pyramid_data_model_agra_COMINT_TargetType_c target;
  uint8_t has_priority;
  pyramid_data_model_agra_ComparableRankingType_c priority;
} pyramid_data_model_agra_COMINT_AcquisitionTargetType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c {
  uint8_t has_target_emitter_data;
  pyramid_data_model_agra_COMINT_AcquisitionTargetType_c target_emitter_data;
  uint8_t has_target_location_data;
  pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c target_location_data;
  pyramid_slice_t frequency_restriction;
} pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c {
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c {
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c {
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  int32_t geolocation_state;
  pyramid_slice_t geolocation_type;
} pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c;

typedef struct pyramid_data_model_agra_COMINT_InteractiveType_c {
  uint8_t has_threshold;
  double threshold;
  uint8_t has_noise_adaptive_threshold_status;
  int32_t noise_adaptive_threshold_status;
  uint8_t has_gain_control_method;
  int32_t gain_control_method;
  uint8_t has_gain;
  double gain;
  pyramid_slice_t center_frequency;
  pyramid_slice_t scan_range;
  pyramid_slice_t scan_lockout;
  uint8_t has_dwell_target;
  pyramid_data_model_agra_COMINT_AcquisitionTargetType_c dwell_target;
  pyramid_slice_t ignore_target;
  pyramid_slice_t technique_in_use;
  pyramid_slice_t technique_parameters;
  uint8_t has_audio_enable;
  uint8_t audio_enable;
  uint8_t has_iq_enable;
  uint8_t iq_enable;
} pyramid_data_model_agra_COMINT_InteractiveType_c;

typedef struct pyramid_data_model_agra_COMINT_DataCollectCommandType_c {
  pyramid_data_model_agra_CollectionFrequencyType_c collection_frequency;
  pyramid_str_t start_time;
  pyramid_str_t dwell_duration;
  uint8_t has_dwells;
  uint32_t dwells;
  pyramid_str_t repetition_interval;
  uint8_t has_signal_bandwidth;
  pyramid_data_model_agra_FrequencyRangeType_c signal_bandwidth;
  uint8_t has_modulation;
  int32_t modulation;
  uint8_t has_data_collect_type;
  int32_t data_collect_type;
  uint8_t has_sample_rate;
  float sample_rate;
  uint8_t has_fft_points_per_sample;
  int32_t fft_points_per_sample;
  uint8_t has_comint_interactive;
  pyramid_data_model_agra_COMINT_InteractiveType_c comint_interactive;
} pyramid_data_model_agra_COMINT_DataCollectCommandType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c {
  pyramid_data_model_agra_COMINT_DataCollectCommandType_c collection;
  uint8_t has_target;
  pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c target;
} pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c;

typedef struct pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c {
  uint8_t has_acquisition;
  pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c acquisition;
  uint8_t has_identification;
  pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c identification;
  uint8_t has_geolocation;
  pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c geolocation;
  uint8_t has_measurement;
  pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c measurement;
  uint8_t has_data_collect;
  pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c data_collect;
} pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c;

typedef struct pyramid_data_model_agra_COMINT_TaskType_c {
  int32_t sub_capability_type;  /* from COMINT_TaskBaseType */
  pyramid_slice_t output;  /* from COMINT_TaskBaseType */
  pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c sub_capability_details;
} pyramid_data_model_agra_COMINT_TaskType_c;

typedef struct pyramid_data_model_agra_CS_SignalType_c {
  pyramid_data_model_agra_ForeignKeyType_c signal;
  pyramid_data_model_agra_FrequencyRangeType_c uplink_frequency;
  int32_t uplink_polarization;
  pyramid_data_model_agra_FrequencyRangeType_c downlink_frequency;
  int32_t downlink_polarization;
  pyramid_slice_t existing_signal;
} pyramid_data_model_agra_CS_SignalType_c;

typedef struct pyramid_data_model_agra_CS_SubDetailDataType_c {
  pyramid_data_model_agra_ForeignKeyType_c sub_detail_identifier;
  pyramid_data_model_agra_FileLocationID_Type_c sub_detail_data_file_id;
} pyramid_data_model_agra_CS_SubDetailDataType_c;

typedef struct pyramid_data_model_agra_CS_DetailDataType_c {
  pyramid_str_t name;
  uint8_t has_details_id;
  pyramid_data_model_agra_FileLocationID_Type_c details_id;
  uint8_t has_signal;
  pyramid_data_model_agra_ForeignKeyType_c signal;
  uint8_t has_uplink_frequency;
  double uplink_frequency;
  uint8_t has_uplink_polarization;
  int32_t uplink_polarization;
  uint8_t has_downlink_frequency;
  double downlink_frequency;
  uint8_t has_downlink_polarization;
  int32_t downlink_polarization;
  uint8_t has_modulation;
  int32_t modulation;
  uint8_t has_bandwidth;
  double bandwidth;
  uint8_t has_attributes_id;
  pyramid_data_model_agra_FileLocationID_Type_c attributes_id;
  pyramid_slice_t sub_detail_data;
} pyramid_data_model_agra_CS_DetailDataType_c;

typedef struct pyramid_data_model_agra_CS_EngagementDataType_c {
  pyramid_str_t seno;
  pyramid_data_model_agra_SystemID_Type_c system_id;
  pyramid_data_model_agra_SatelliteIdentifierType_c satellite;
  pyramid_data_model_agra_ForeignKeyType_c transponder;
  pyramid_data_model_agra_ForeignKeyType_c tactic;
  uint8_t has_comment_id;
  pyramid_data_model_agra_FileLocationID_Type_c comment_id;
  pyramid_data_model_agra_CS_DetailDataType_c detail_data;
  pyramid_slice_t signals;
} pyramid_data_model_agra_CS_EngagementDataType_c;

typedef struct pyramid_data_model_agra_CounterSpaceTaskType_c {
  uint8_t has_force_name;
  pyramid_data_model_agra_ForeignKeyType_c force_name;
  pyramid_data_model_agra_CS_EngagementDataType_c engagement_data;
} pyramid_data_model_agra_CounterSpaceTaskType_c;

typedef struct pyramid_data_model_agra_AO_TaskType_c {
  int32_t capability_type;  /* from AO_TaskBaseType */
  uint8_t has_supported_code;
  pyramid_data_model_agra_AO_CodeType_c supported_code;  /* from AO_TaskBaseType */
  uint8_t has_emission_constraints;
  pyramid_data_model_agra_OpticalCollectionConstraintsType_c emission_constraints;  /* from AO_TaskBaseType */
  pyramid_data_model_agra_TargetType_c target;
} pyramid_data_model_agra_AO_TaskType_c;

typedef struct pyramid_data_model_agra_AirSampleTaskType_c {
  int32_t capability_type;  /* from AirSampleTaskBaseType */
  pyramid_slice_t output;  /* from AirSampleTaskBaseType */
  pyramid_data_model_agra_TargetType_c target;
} pyramid_data_model_agra_AirSampleTaskType_c;

typedef struct pyramid_data_model_agra_LocationType_c {
  uint8_t has_path;
  pyramid_data_model_agra_PathType_c path;
  uint8_t has_loiter;
  pyramid_data_model_agra_LoiterType_c loiter;
  uint8_t has_zone;
  pyramid_data_model_agra_ZoneType_c zone;
} pyramid_data_model_agra_LocationType_c;

typedef struct pyramid_data_model_agra_CommRelayTaskType_c {
  int32_t capability_type;  /* from CommRelayTaskBaseType */
  pyramid_slice_t frequency_set;  /* from CommRelayTaskBaseType */
  uint8_t has_location;
  pyramid_data_model_agra_LocationType_c location;
  pyramid_slice_t relay_node;
} pyramid_data_model_agra_CommRelayTaskType_c;

typedef struct pyramid_data_model_agra_MA_CAPOffsetType_c {
  uint8_t has_time;
  pyramid_data_model_agra_DurationRangeType_c time;
  uint8_t has_distance;
  double distance;
  uint8_t has_percentof_pattern;
  double percentof_pattern;
} pyramid_data_model_agra_MA_CAPOffsetType_c;

typedef struct pyramid_data_model_agra_MA_CAPReferencePointType_c {
  int32_t leg_index;
  pyramid_data_model_agra_MA_CAPOffsetType_c arrival_offset;
} pyramid_data_model_agra_MA_CAPReferencePointType_c;

typedef struct pyramid_data_model_agra_MA_CAPRelativePositionType_c {
  pyramid_data_model_agra_SystemID_Type_c asset_id;
  pyramid_data_model_agra_SystemID_Type_c reference_id;
  pyramid_slice_t reference_point;
} pyramid_data_model_agra_MA_CAPRelativePositionType_c;

typedef struct pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c;

typedef struct pyramid_data_model_agra_MA_SynchronizationChoiceType_c {
  uint8_t has_strategy;
  int32_t strategy;
  uint8_t has_relative_positioning;
  pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c relative_positioning;
} pyramid_data_model_agra_MA_SynchronizationChoiceType_c;

typedef struct pyramid_data_model_agra_PathSegmentSpeedValueType_c {
  double value;
  int32_t reference;
} pyramid_data_model_agra_PathSegmentSpeedValueType_c;

typedef struct pyramid_data_model_agra_PathSegmentSpeedChoiceType_c {
  uint8_t has_speed_value;
  pyramid_data_model_agra_PathSegmentSpeedValueType_c speed_value;
  uint8_t has_mach_value;
  double mach_value;
} pyramid_data_model_agra_PathSegmentSpeedChoiceType_c;

typedef struct pyramid_data_model_agra_PathSegmentSpeedType_c {
  uint8_t has_speed_choice;
  pyramid_data_model_agra_PathSegmentSpeedChoiceType_c speed_choice;
  uint8_t has_speed_optimization;
  int32_t speed_optimization;
} pyramid_data_model_agra_PathSegmentSpeedType_c;

typedef struct pyramid_data_model_agra_MA_CircleOrbitType_c {
  int32_t orbit_type;
  pyramid_slice_t circle;
  int32_t turn_direction;
} pyramid_data_model_agra_MA_CircleOrbitType_c;

typedef struct pyramid_data_model_agra_IntervalChoiceType_c {
  uint8_t has_distance;
  double distance;
  uint8_t has_duration;
  pyramid_str_t duration;
} pyramid_data_model_agra_IntervalChoiceType_c;

typedef struct pyramid_data_model_agra_MA_DirectionReferenceType_c {
  double value;
  int32_t reference;
} pyramid_data_model_agra_MA_DirectionReferenceType_c;

typedef struct pyramid_data_model_agra_MA_DirectionChoiceType_c {
  uint8_t has_heading;
  pyramid_data_model_agra_MA_DirectionReferenceType_c heading;
  uint8_t has_course;
  pyramid_data_model_agra_MA_DirectionReferenceType_c course;
} pyramid_data_model_agra_MA_DirectionChoiceType_c;

typedef struct pyramid_data_model_agra_TurnGeometryChoiceType_c {
  uint8_t has_turn_radius;
  double turn_radius;
  uint8_t has_bank_angle;
  double bank_angle;
} pyramid_data_model_agra_TurnGeometryChoiceType_c;

typedef struct pyramid_data_model_agra_MA_FixOrbitType_c {
  pyramid_data_model_agra_PointChoiceType_c point;
  pyramid_data_model_agra_MA_DirectionChoiceType_c inbound_direction;
  pyramid_data_model_agra_IntervalChoiceType_c leg_length;
  pyramid_data_model_agra_TurnGeometryChoiceType_c turn_geometry;
  int32_t turn_direction;
} pyramid_data_model_agra_MA_FixOrbitType_c;

typedef struct pyramid_data_model_agra_MA_OrbitShapeType_c {
  uint8_t has_fix_point;
  pyramid_data_model_agra_MA_FixOrbitType_c fix_point;
  uint8_t has_circle;
  pyramid_data_model_agra_MA_CircleOrbitType_c circle;
} pyramid_data_model_agra_MA_OrbitShapeType_c;

typedef struct pyramid_data_model_agra_MA_OrbitType_c {
  pyramid_data_model_agra_MA_OrbitShapeType_c orbit_shape;
  uint8_t has_duration;
  pyramid_data_model_agra_OrbitDurationType_c duration;
  uint8_t has_entry_point;
  pyramid_data_model_agra_PointChoiceType_c entry_point;
  uint8_t has_exit_point;
  pyramid_data_model_agra_PointChoiceType_c exit_point;
} pyramid_data_model_agra_MA_OrbitType_c;

typedef struct pyramid_data_model_agra_MA_CAP_TaskType_c {
  pyramid_slice_t orbits;
  uint8_t has_assets_per_orbit;
  uint32_t assets_per_orbit;
  uint8_t has_optimization_strategy;
  int32_t optimization_strategy;
  uint8_t has_synchronization_strategy;
  pyramid_data_model_agra_MA_SynchronizationChoiceType_c synchronization_strategy;
  pyramid_slice_t keepin_zone_id;
  pyramid_slice_t hostile_zone_id;
  uint8_t has_cap_speed;
  pyramid_data_model_agra_PathSegmentSpeedType_c cap_speed;
} pyramid_data_model_agra_MA_CAP_TaskType_c;

typedef struct pyramid_data_model_agra_RSO_ApproachType_c {
  pyramid_data_model_agra_AssetType_c rso;
  uint8_t has_range;
  pyramid_data_model_agra_DistanceConstraintsType_c range;
} pyramid_data_model_agra_RSO_ApproachType_c;

typedef struct pyramid_data_model_agra_OrbitalVolumeType_c {
  pyramid_data_model_agra_OrbitalKinematicsChoiceType_c position;
  double radius;
} pyramid_data_model_agra_OrbitalVolumeType_c;

typedef struct pyramid_data_model_agra_COE_OrbitType_c {
  double eccentricity;  /* from COE_OrbitBaseType */
  double inclination;  /* from COE_OrbitBaseType */
  pyramid_data_model_agra_COE_OrientationType_c orientation;  /* from COE_OrbitBaseType */
  double semimajor_axis;
} pyramid_data_model_agra_COE_OrbitType_c;

typedef struct pyramid_data_model_agra_ProximityOrbitChoiceType_c {
  uint8_t has_natural_motion;
  pyramid_str_t natural_motion;
  uint8_t has_forced_motion;
  pyramid_str_t forced_motion;
  uint8_t has_r_bar_perch;
  int32_t r_bar_perch;
  uint8_t has_v_bar_perch;
  int32_t v_bar_perch;
  uint8_t has_delta_orbital_plane_tolerance;
  pyramid_data_model_agra_AngleHalfPairType_c delta_orbital_plane_tolerance;
} pyramid_data_model_agra_ProximityOrbitChoiceType_c;

typedef struct pyramid_data_model_agra_ProximityOperationsType_c {
  pyramid_data_model_agra_AssetType_c rso;  /* from RSO_ApproachType */
  uint8_t has_range;
  pyramid_data_model_agra_DistanceConstraintsType_c range;  /* from RSO_ApproachType */
  uint8_t has_proximity_orbit;
  pyramid_data_model_agra_ProximityOrbitChoiceType_c proximity_orbit;
} pyramid_data_model_agra_ProximityOperationsType_c;

typedef struct pyramid_data_model_agra_RaceTrackOrbitType_c {
  pyramid_str_t duration;
  double start_longitude;
  double end_longitude;
} pyramid_data_model_agra_RaceTrackOrbitType_c;

typedef struct pyramid_data_model_agra_OrbitChangeChoiceType_c {
  uint8_t has_specific_orbit;
  pyramid_data_model_agra_COE_OrbitType_c specific_orbit;
  uint8_t has_specific_position;
  pyramid_data_model_agra_OrbitalVolumeType_c specific_position;
  uint8_t has_rendezvous;
  pyramid_data_model_agra_RSO_ApproachType_c rendezvous;
  uint8_t has_proximity_operations;
  pyramid_data_model_agra_ProximityOperationsType_c proximity_operations;
  uint8_t has_longitude;
  double longitude;
  uint8_t has_semimajor_axis;
  double semimajor_axis;
  uint8_t has_inclination;
  double inclination;
  uint8_t has_eccentricity;
  double eccentricity;
  uint8_t has_race_track;
  pyramid_data_model_agra_RaceTrackOrbitType_c race_track;
} pyramid_data_model_agra_OrbitChangeChoiceType_c;

typedef struct pyramid_data_model_agra_DisposalOrbitType_c {
  uint8_t passivate;
} pyramid_data_model_agra_DisposalOrbitType_c;

typedef struct pyramid_data_model_agra_OrbitChangeTaskType_c {
  int32_t capability_type;  /* from OrbitChangeTaskBaseType */
  pyramid_data_model_agra_OrbitChangeChoiceType_c change_type;
  uint8_t has_disposal_orbit;
  pyramid_data_model_agra_DisposalOrbitType_c disposal_orbit;
} pyramid_data_model_agra_OrbitChangeTaskType_c;

typedef struct pyramid_data_model_agra_AMTI_TargetType_c {
  uint8_t has_air_volume_sensor_referenced;
  pyramid_data_model_agra_AirVolumeSensorReferencedType_c air_volume_sensor_referenced;
  uint8_t has_air_volume_location;
  pyramid_data_model_agra_ZoneType_c air_volume_location;
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_AMTI_TargetType_c;

typedef struct pyramid_data_model_agra_AMTI_TaskType_c {
  int32_t capability_type;  /* from AMTI_TaskBaseType */
  pyramid_slice_t sub_capability_type;  /* from AMTI_TaskBaseType */
  uint8_t has_collection_policy;
  int32_t collection_policy;  /* from AMTI_TaskBaseType */
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_AMTI_CollectionConstraintsType_c collection_constraints;  /* from AMTI_TaskBaseType */
  pyramid_slice_t output;  /* from AMTI_TaskBaseType */
  pyramid_slice_t target;
} pyramid_data_model_agra_AMTI_TaskType_c;

typedef struct pyramid_data_model_agra_TacticalOrderTaskType_c {
  int32_t capability_type;  /* from TacticalOrderTaskBaseType */
  uint8_t has_target;
  pyramid_data_model_agra_TargetType_c target;
} pyramid_data_model_agra_TacticalOrderTaskType_c;

typedef struct pyramid_data_model_agra_SectorType_c {
  double min;  /* from AnglePairType */
  double max;  /* from AnglePairType */
  double range;
} pyramid_data_model_agra_SectorType_c;

typedef struct pyramid_data_model_agra_CargoID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_CargoID_Type_c;

typedef struct pyramid_data_model_agra_CargoTransitionType_c {
  pyramid_slice_t cargo_id;
  int32_t vehicle_control_strategy;
  int32_t load_unload_strategy;
  pyramid_slice_t transition_location;
  uint8_t has_egress_constraint;
  pyramid_data_model_agra_SectorType_c egress_constraint;
  uint8_t has_ingress_constraint;
  pyramid_data_model_agra_SectorType_c ingress_constraint;
} pyramid_data_model_agra_CargoTransitionType_c;

typedef struct pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c;

typedef struct pyramid_data_model_agra_CargoDeliveryTaskType_c {
  uint8_t has_pickup;
  pyramid_data_model_agra_CargoTransitionType_c pickup;
  uint8_t has_dropoff;
  pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c dropoff;
} pyramid_data_model_agra_CargoDeliveryTaskType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c {
  pyramid_data_model_agra_GeocentricVolumeType_c geocentric_zone;
} pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceTargetType_c {
  uint8_t has_object_based;
  pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c object_based;
  uint8_t has_location_based;
  pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c location_based;
  uint8_t has_zone_based;
  pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c zone_based;
} pyramid_data_model_agra_OrbitalSurveillanceTargetType_c;

typedef struct pyramid_data_model_agra_OrbitalSurveillanceTaskType_c {
  int32_t capability_type;  /* from OrbitalSurveillanceTaskBaseType */
  pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c sub_capability_type;  /* from OrbitalSurveillanceTaskBaseType */
  uint8_t has_information_objective;
  int32_t information_objective;  /* from OrbitalSurveillanceTaskBaseType */
  uint8_t has_min_size;
  double min_size;  /* from OrbitalSurveillanceTaskBaseType */
  uint8_t confirm_object_acquisition;  /* from OrbitalSurveillanceTaskBaseType */
  uint8_t has_product_needed_by;
  pyramid_data_model_agra_ProductNeededByType_c product_needed_by;  /* from OrbitalSurveillanceTaskBaseType */
  uint8_t has_sensor_constraints;
  pyramid_data_model_agra_AllowableSensorsType_c sensor_constraints;  /* from OrbitalSurveillanceTaskBaseType */
  pyramid_slice_t sda_special_instructions;  /* from OrbitalSurveillanceTaskBaseType */
  pyramid_data_model_agra_OrbitalSurveillanceTargetType_c target;
} pyramid_data_model_agra_OrbitalSurveillanceTaskType_c;

typedef struct pyramid_data_model_agra_SAR_TargetType_c {
  pyramid_data_model_agra_TargetType_c target;
  uint8_t has_multi_look;
  uint8_t multi_look;
} pyramid_data_model_agra_SAR_TargetType_c;

typedef struct pyramid_data_model_agra_ISAR_TargetType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_raw_target;
  pyramid_data_model_agra_PointTargetType_c raw_target;
} pyramid_data_model_agra_ISAR_TargetType_c;

typedef struct pyramid_data_model_agra_SAR_TaskTargetType_c {
  uint8_t has_sar;
  pyramid_data_model_agra_SAR_TargetType_c sar;
  uint8_t has_isar;
  pyramid_data_model_agra_ISAR_TargetType_c isar;
} pyramid_data_model_agra_SAR_TaskTargetType_c;

typedef struct pyramid_data_model_agra_SAR_TaskType_c {
  int32_t capability_type;  /* from SAR_TaskBaseType */
  int32_t sub_capability_type;  /* from SAR_TaskBaseType */
  uint8_t has_look_at_coords;
  pyramid_data_model_agra_LocatedEllipseType_c look_at_coords;  /* from SAR_TaskBaseType */
  uint8_t has_collection_options;
  pyramid_data_model_agra_SAR_CollectionOptionsType_c collection_options;  /* from SAR_TaskBaseType */
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_SAR_CollectionConstraintsType_c collection_constraints;  /* from SAR_TaskBaseType */
  uint8_t has_desired_waveform;
  pyramid_data_model_agra_SAR_WaveformType_c desired_waveform;  /* from SAR_TaskBaseType */
  uint8_t has_pair_identifier;
  int32_t pair_identifier;  /* from SAR_TaskBaseType */
  pyramid_slice_t output;  /* from SAR_TaskBaseType */
  pyramid_data_model_agra_SAR_TaskTargetType_c target;
} pyramid_data_model_agra_SAR_TaskType_c;

typedef struct pyramid_data_model_agra_MA_FlightTaskBaseType_c {
  pyramid_slice_t capability_type;
} pyramid_data_model_agra_MA_FlightTaskBaseType_c;

typedef struct pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c {
  pyramid_data_model_agra_LoiterType_c loiter_kinematics;
  double minimum_altitude;
  uint8_t has_maximum_altitude;
  double maximum_altitude;
  uint8_t has_minimum_altitude_separation;
  double minimum_altitude_separation;
} pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c;

typedef struct pyramid_data_model_agra_MA_HoldTurnSpecificationType_c {
  uint8_t has_turn_radius;
  double turn_radius;
  uint8_t has_turn_rate;
  double turn_rate;
  uint8_t has_turn_type;
  int32_t turn_type;
} pyramid_data_model_agra_MA_HoldTurnSpecificationType_c;

typedef struct pyramid_data_model_agra_MA_OrbitDurationType_c {
  uint8_t has_time;
  pyramid_str_t time;
  uint8_t has_number_of_orbits;
  uint32_t number_of_orbits;
  uint8_t has_entry_exit_time;
  pyramid_data_model_agra_DateTimeRangeType_c entry_exit_time;
} pyramid_data_model_agra_MA_OrbitDurationType_c;

typedef struct pyramid_data_model_agra_MA_HoldLegSpecificationType_c {
  uint8_t has_leg_time;
  pyramid_str_t leg_time;
  uint8_t has_leg_length;
  double leg_length;
} pyramid_data_model_agra_MA_HoldLegSpecificationType_c;

typedef struct pyramid_data_model_agra_MA_HoldType_c {
  pyramid_data_model_agra_Point2D_Type_c anchor_point;
  uint8_t has_hold_context;
  int32_t hold_context;
  uint8_t has_speed;
  pyramid_data_model_agra_PathSegmentSpeedType_c speed;
  uint8_t has_orientation;
  double orientation;
  uint8_t has_turn_direction;
  int32_t turn_direction;
  pyramid_data_model_agra_MA_HoldLegSpecificationType_c leg_specification;
  pyramid_data_model_agra_MA_HoldTurnSpecificationType_c turn_specification;
  pyramid_data_model_agra_MA_OrbitDurationType_c duration;
  int32_t entry_type;
} pyramid_data_model_agra_MA_HoldType_c;

typedef struct pyramid_data_model_agra_MA_LoiterType_c {
  uint8_t has_orbit;
  pyramid_data_model_agra_MA_OrbitType_c orbit;
  uint8_t has_hover;
  pyramid_data_model_agra_HoverType_c hover;
  uint8_t has_hold;
  pyramid_data_model_agra_MA_HoldType_c hold;
} pyramid_data_model_agra_MA_LoiterType_c;

typedef struct pyramid_data_model_agra_MA_FormationAnchorType_c {
  pyramid_data_model_agra_ReferenceFrameID_Type_c reference_frame_id;
} pyramid_data_model_agra_MA_FormationAnchorType_c;

typedef struct pyramid_data_model_agra_MA_FormationType_c {
  int32_t formation_template;
  pyramid_data_model_agra_MA_FormationAnchorType_c formation_anchor;
  int32_t formation_slot;
  pyramid_data_model_agra_ZoneExternalType_c zone_target;
} pyramid_data_model_agra_MA_FormationType_c;

typedef struct pyramid_data_model_agra_AirfieldID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_AirfieldID_Type_c;

typedef struct pyramid_data_model_agra_RunwayID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_RunwayID_Type_c;

typedef struct pyramid_data_model_agra_MA_AirfieldLandType_c {
  pyramid_data_model_agra_AirfieldID_Type_c airfield_id;
  pyramid_data_model_agra_RunwayID_Type_c runway_id;
} pyramid_data_model_agra_MA_AirfieldLandType_c;

typedef struct pyramid_data_model_agra_MA_CarrierRecoveryType_c {
  uint8_t has_case_designation;
  int32_t case_designation;
  double final_bearing;
  double glideslope;
  pyramid_data_model_agra_Point3D_RelativeType_c relative_touch_down_point;
  pyramid_data_model_agra_SystemID_Type_c carrier_id;
  uint8_t has_carrier_playbox_id;
  pyramid_data_model_agra_OpZoneID_Type_c carrier_playbox_id;
  pyramid_str_t expected_arrival_time;
} pyramid_data_model_agra_MA_CarrierRecoveryType_c;

typedef struct pyramid_data_model_agra_MA_DeltaType_c {
  pyramid_str_t time_delta_seconds;
} pyramid_data_model_agra_MA_DeltaType_c;

typedef struct pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c {
  uint8_t has_recovery;
  pyramid_data_model_agra_MA_CarrierRecoveryType_c recovery;
  uint8_t has_delta;
  pyramid_data_model_agra_MA_DeltaType_c delta;
} pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c;

typedef struct pyramid_data_model_agra_MA_RecoveryType_c {
  uint8_t has_carrier_recovery;
  pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c carrier_recovery;
  uint8_t has_airfield_land;
  pyramid_data_model_agra_MA_AirfieldLandType_c airfield_land;
} pyramid_data_model_agra_MA_RecoveryType_c;

typedef struct pyramid_data_model_agra_MA_HSA_CSA_Type_c {
  uint8_t has_altitude;
  pyramid_data_model_agra_AltitudeReferenceType_c altitude;
  uint8_t has_speed;
  pyramid_data_model_agra_PathSegmentSpeedType_c speed;
  uint8_t has_direction;
  pyramid_data_model_agra_MA_DirectionChoiceType_c direction;
} pyramid_data_model_agra_MA_HSA_CSA_Type_c;

typedef struct pyramid_data_model_agra_MA_CurveTraversingType_c {
  uint8_t has_speed_range;
  pyramid_data_model_agra_SpeedRangeType_c speed_range;
  uint8_t has_duration;
  pyramid_str_t duration;
} pyramid_data_model_agra_MA_CurveTraversingType_c;

typedef struct pyramid_data_model_agra_MA_NURBS_ControlPointType_c {
  pyramid_data_model_agra_Point2D_RelativeType_c control_point;
  double weight;
} pyramid_data_model_agra_MA_NURBS_ControlPointType_c;

typedef struct pyramid_data_model_agra_MA_NURBS_PointType_c {
  pyramid_data_model_agra_WayPointPointChoiceType_c center_reference;
  pyramid_slice_t control_points;
  pyramid_slice_t knot_vector;
  uint8_t has_curve_traversing_parameters;
  pyramid_data_model_agra_MA_CurveTraversingType_c curve_traversing_parameters;
  uint8_t has_curvature;
  double curvature;
  uint8_t has_initial_control_point_index;
  uint32_t initial_control_point_index;
  uint8_t has_final_control_point_index;
  uint32_t final_control_point_index;
} pyramid_data_model_agra_MA_NURBS_PointType_c;

typedef struct pyramid_data_model_agra_MA_CurveControlType_c {
  pyramid_slice_t curve_segments;
  pyramid_str_t append_curve;
  uint8_t has_end_of_curve_behavior;
  int32_t end_of_curve_behavior;
} pyramid_data_model_agra_MA_CurveControlType_c;

typedef struct pyramid_data_model_agra_CF_CourseToFixType_c {
  double course;
} pyramid_data_model_agra_CF_CourseToFixType_c;

typedef struct pyramid_data_model_agra_RF_RadiusToFixType_c {
  pyramid_data_model_agra_Point2D_Type_c radius_center_point;
  double turn_radius;
  double course_in;
  double course_out;
  pyramid_data_model_agra_Point2D_Type_c arc_initial_point;
  pyramid_data_model_agra_Point2D_Type_c arc_end_point;
  double arc_distance;
  double arc_direct_distance;
  int32_t turn_direction;
} pyramid_data_model_agra_RF_RadiusToFixType_c;

typedef struct pyramid_data_model_agra_CivilPathTerminatorType_c {
  uint8_t has_af_arc_to_fix;
  pyramid_str_t af_arc_to_fix;
  uint8_t has_ca_course_to_altitude;
  pyramid_str_t ca_course_to_altitude;
  uint8_t has_cd_course_to_dme_distance;
  pyramid_str_t cd_course_to_dme_distance;
  uint8_t has_cf_course_to_fix;
  pyramid_data_model_agra_CF_CourseToFixType_c cf_course_to_fix;
  uint8_t has_ci_course_to_intercept;
  pyramid_str_t ci_course_to_intercept;
  uint8_t has_cr_course_to_radial;
  pyramid_str_t cr_course_to_radial;
  uint8_t has_df_direct_to_fix;
  pyramid_str_t df_direct_to_fix;
  uint8_t has_fa_track_to_altitude;
  pyramid_str_t fa_track_to_altitude;
  uint8_t has_fc_track_from_fix_to_distance_along_track;
  pyramid_str_t fc_track_from_fix_to_distance_along_track;
  uint8_t has_fd_track_from_fix_to_dme_distance;
  pyramid_str_t fd_track_from_fix_to_dme_distance;
  uint8_t has_fm_fix_to_manual_termination;
  pyramid_str_t fm_fix_to_manual_termination;
  uint8_t has_ha_holding_with_altitude_termination;
  pyramid_str_t ha_holding_with_altitude_termination;
  uint8_t has_hf_holding_with_fix_termination;
  pyramid_str_t hf_holding_with_fix_termination;
  uint8_t has_hm_holding_with_manual_termination;
  pyramid_str_t hm_holding_with_manual_termination;
  uint8_t has_if_initial_fix;
  pyramid_str_t if_initial_fix;
  uint8_t has_pi_procedure_turn_to_intercept;
  pyramid_str_t pi_procedure_turn_to_intercept;
  uint8_t has_rf_radius_to_fix;
  pyramid_data_model_agra_RF_RadiusToFixType_c rf_radius_to_fix;
  uint8_t has_tf_track_to_fix;
  pyramid_str_t tf_track_to_fix;
  uint8_t has_va_heading_to_altitude;
  pyramid_str_t va_heading_to_altitude;
  uint8_t has_vd_heading_to_dme_distance_termination;
  pyramid_str_t vd_heading_to_dme_distance_termination;
  uint8_t has_vi_heading_to_intercept;
  pyramid_str_t vi_heading_to_intercept;
  uint8_t has_vm_heading_to_manual;
  pyramid_str_t vm_heading_to_manual;
  uint8_t has_vr_heading_to_radial_termination;
  pyramid_str_t vr_heading_to_radial_termination;
} pyramid_data_model_agra_CivilPathTerminatorType_c;

typedef struct pyramid_data_model_agra_MA_LoiterPointType_c {
  pyramid_data_model_agra_MA_LoiterType_c loiter;
  pyramid_str_t end_time;
} pyramid_data_model_agra_MA_LoiterPointType_c;

typedef struct pyramid_data_model_agra_TurnPointType_c {
  pyramid_data_model_agra_WayPointPointChoiceType_c waypoint;
  int32_t turn_point_type;
  uint8_t has_course;
  double course;
  uint8_t has_turn_geometry;
  pyramid_data_model_agra_TurnGeometryChoiceType_c turn_geometry;
} pyramid_data_model_agra_TurnPointType_c;

typedef struct pyramid_data_model_agra_MA_EndPointType_c {
  uint8_t has_way_point;
  pyramid_data_model_agra_WayPointType_c way_point;
  uint8_t has_turn_point;
  pyramid_data_model_agra_TurnPointType_c turn_point;
  uint8_t has_loiter_point;
  pyramid_data_model_agra_MA_LoiterPointType_c loiter_point;
} pyramid_data_model_agra_MA_EndPointType_c;

typedef struct pyramid_data_model_agra_NextPathSegmentType_c {
  uint8_t has_path_id;
  pyramid_data_model_agra_PathID_Type_c path_id;
  pyramid_data_model_agra_SegmentID_Type_c path_segment_id;
} pyramid_data_model_agra_NextPathSegmentType_c;

typedef struct pyramid_data_model_agra_ClimbType_c {
  uint8_t has_climb_rate;
  double climb_rate;
  uint8_t has_climb_type;
  int32_t climb_type;
} pyramid_data_model_agra_ClimbType_c;

typedef struct pyramid_data_model_agra_EnduranceRemainingType_c {
  pyramid_data_model_agra_EnduranceType_c endurance_remaining;
  int32_t logical_operator;
} pyramid_data_model_agra_EnduranceRemainingType_c;

typedef struct pyramid_data_model_agra_SegmentCaptureType_c {
  uint32_t capture_count;
  int32_t logical_operator;
} pyramid_data_model_agra_SegmentCaptureType_c;

typedef struct pyramid_data_model_agra_PathSegmentConditionType_c {
  uint8_t has_altitude_range;
  pyramid_data_model_agra_AltitudeRangeType_c altitude_range;
  uint8_t has_time_window;
  pyramid_data_model_agra_DateTimeRangeType_c time_window;
  uint8_t has_segment_capture;
  pyramid_data_model_agra_SegmentCaptureType_c segment_capture;
  uint8_t has_operator_input;
  uint8_t operator_input;
  uint8_t has_endurance;
  pyramid_data_model_agra_EnduranceRemainingType_c endurance;
  uint8_t has_contingency_level;
  int32_t contingency_level;
} pyramid_data_model_agra_PathSegmentConditionType_c;

typedef struct pyramid_data_model_agra_ConditionalPathSegmentType_c {
  uint8_t has_path_id;
  pyramid_data_model_agra_PathID_Type_c path_id;
  pyramid_data_model_agra_SegmentID_Type_c path_segment_id;
  uint8_t has_condition;
  pyramid_data_model_agra_PathSegmentConditionType_c condition;
} pyramid_data_model_agra_ConditionalPathSegmentType_c;

typedef struct pyramid_data_model_agra_MA_PathSegmentType_c {
  pyramid_data_model_agra_SegmentID_Type_c path_segment_id;
  int32_t source;
  pyramid_data_model_agra_MA_EndPointType_c end_point;
  uint8_t has_locked;
  uint8_t locked;
  uint8_t has_modified;
  uint8_t modified;
  uint8_t has_speed;
  pyramid_data_model_agra_PathSegmentSpeedType_c speed;
  uint8_t has_civil_path_terminator;
  pyramid_data_model_agra_CivilPathTerminatorType_c civil_path_terminator;
  uint8_t has_climb;
  pyramid_data_model_agra_ClimbType_c climb;
  uint8_t has_maximum_roll;
  double maximum_roll;
  uint8_t has_acceleration;
  double acceleration;
  uint8_t has_next_path_segment;
  pyramid_data_model_agra_NextPathSegmentType_c next_path_segment;
  pyramid_slice_t conditional_path_segment;
  pyramid_slice_t inertial_state;
  uint8_t has_required_time_of_arrival;
  pyramid_data_model_agra_TimeWindowType_c required_time_of_arrival;
  uint8_t has_remarks;
  pyramid_data_model_agra_RemarksType_c remarks;
  uint8_t has_required_navigation_performance_in_meters;
  double required_navigation_performance_in_meters;
  uint8_t has_fix_identifier;
  pyramid_data_model_agra_ForeignKeyType_c fix_identifier;
} pyramid_data_model_agra_MA_PathSegmentType_c;

typedef struct pyramid_data_model_agra_MA_RoutePathType_c {
  pyramid_data_model_agra_PathID_Type_c path_id;
  int32_t path_type;
  pyramid_data_model_agra_SegmentID_Type_c first_in_path_segment_id;
  pyramid_slice_t path_segment;
  uint8_t has_initial_conditions;
  pyramid_data_model_agra_PlanningLocationType_c initial_conditions;
  uint8_t has_airfield_id;
  pyramid_data_model_agra_AirfieldID_Type_c airfield_id;
  uint8_t has_runway_id;
  pyramid_data_model_agra_RunwayID_Type_c runway_id;
  uint8_t has_remarks;
  pyramid_data_model_agra_RemarksType_c remarks;
} pyramid_data_model_agra_MA_RoutePathType_c;

typedef struct pyramid_data_model_agra_MA_RouteType_c {
  uint8_t detailed;
  pyramid_data_model_agra_PathID_Type_c first_in_route_path_id;
  int32_t route_projection;
  pyramid_slice_t path;
  uint8_t has_remarks;
  pyramid_data_model_agra_RemarksType_c remarks;
} pyramid_data_model_agra_MA_RouteType_c;

typedef struct pyramid_data_model_agra_MA_WaypointFollowingType_c {
  pyramid_data_model_agra_MA_RouteType_c route;
} pyramid_data_model_agra_MA_WaypointFollowingType_c;

typedef struct pyramid_data_model_agra_MA_FlightControlModesChoiceType_c {
  uint8_t has_hsa_csa;
  pyramid_data_model_agra_MA_HSA_CSA_Type_c hsa_csa;
  uint8_t has_waypoint_following;
  pyramid_data_model_agra_MA_WaypointFollowingType_c waypoint_following;
  uint8_t has_curve_following;
  pyramid_data_model_agra_MA_CurveControlType_c curve_following;
} pyramid_data_model_agra_MA_FlightControlModesChoiceType_c;

typedef struct pyramid_data_model_agra_MA_CatapultID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MA_CatapultID_Type_c;

typedef struct pyramid_data_model_agra_MA_CarrierLaunchType_c {
  uint8_t has_case_designation;
  int32_t case_designation;
  pyramid_data_model_agra_OpLineID_Type_c departure_radial_line_id;
  pyramid_data_model_agra_SystemID_Type_c carrier_id;
  pyramid_data_model_agra_MA_CatapultID_Type_c catapult_id;
} pyramid_data_model_agra_MA_CarrierLaunchType_c;

typedef struct pyramid_data_model_agra_MA_AirfieldTakeoffType_c {
  pyramid_data_model_agra_AirfieldID_Type_c airfield_id;
  pyramid_data_model_agra_RunwayID_Type_c runway_id;
} pyramid_data_model_agra_MA_AirfieldTakeoffType_c;

typedef struct pyramid_data_model_agra_MA_LaunchType_c {
  uint8_t has_carrier_launch;
  pyramid_data_model_agra_MA_CarrierLaunchType_c carrier_launch;
  uint8_t has_airfield_takeoff;
  pyramid_data_model_agra_MA_AirfieldTakeoffType_c airfield_takeoff;
} pyramid_data_model_agra_MA_LaunchType_c;

typedef struct pyramid_data_model_agra_MustFlyLocationType_c {
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
  uint8_t has_op_point_id;
  pyramid_data_model_agra_OpPointID_Type_c op_point_id;
  uint8_t has_op_line_id;
  pyramid_data_model_agra_OpLineID_Type_c op_line_id;
  uint8_t has_op_zone_id;
  pyramid_data_model_agra_OpZoneID_Type_c op_zone_id;
  uint8_t has_op_volume_id;
  pyramid_data_model_agra_OpVolumeID_Type_c op_volume_id;
  uint8_t has_point;
  pyramid_data_model_agra_Point3D_Type_c point;
  uint8_t has_zone_target;
  pyramid_data_model_agra_ZoneExternalType_c zone_target;
  uint8_t has_line_target;
  pyramid_data_model_agra_LineTargetType_c line_target;
  uint8_t has_volume_target;
  pyramid_data_model_agra_OpVolumeType_c volume_target;
} pyramid_data_model_agra_MustFlyLocationType_c;

typedef struct pyramid_data_model_agra_MustFlyType_c {
  pyramid_data_model_agra_MustFlyLocationType_c location;
  uint8_t has_ingress_constraint;
  pyramid_data_model_agra_AnglePairType_c ingress_constraint;
} pyramid_data_model_agra_MustFlyType_c;

typedef struct pyramid_data_model_agra_MA_FlightTaskType_c {
  pyramid_slice_t capability_type;  /* from MA_FlightTaskBaseType */
  uint8_t has_loiter;
  pyramid_data_model_agra_MA_LoiterType_c loiter;
  uint8_t has_must_fly;
  pyramid_data_model_agra_MustFlyType_c must_fly;
  pyramid_slice_t formation;
  uint8_t has_altitude_stacked_marshall;
  pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c altitude_stacked_marshall;
  uint8_t has_flight_control_mode;
  pyramid_data_model_agra_MA_FlightControlModesChoiceType_c flight_control_mode;
  uint8_t has_launch;
  pyramid_data_model_agra_MA_LaunchType_c launch;
  uint8_t has_recovery;
  pyramid_data_model_agra_MA_RecoveryType_c recovery;
} pyramid_data_model_agra_MA_FlightTaskType_c;

typedef struct pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c {
  double azimuth_scan_width;
  double azimuth_scan_center;
  double elevation_scan_width;
  double elevation_scan_center;
  double max_range_of_interest;
  double min_range_of_interest;
  int32_t azimuth_scan_stabilization;
  int32_t elevation_scan_stabilization;
  uint8_t has_elevation_scan_center_altitude_range_pair;
  pyramid_data_model_agra_AltitudeRangePairType_c elevation_scan_center_altitude_range_pair;
  uint8_t roll_stabilized;
} pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c;

typedef struct pyramid_data_model_agra_PO_AirTargetVolumeType_c {
  uint8_t has_air_volume_sensor_referenced;
  pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c air_volume_sensor_referenced;
  uint8_t has_air_volume_location;
  pyramid_data_model_agra_ZoneType_c air_volume_location;
} pyramid_data_model_agra_PO_AirTargetVolumeType_c;

typedef struct pyramid_data_model_agra_PO_AirTargetVolumeCommandType_c {
  uint8_t has_air_volume;
  pyramid_data_model_agra_PO_AirTargetVolumeType_c air_volume;
  uint8_t has_entity_id;
  pyramid_data_model_agra_EntityID_Type_c entity_id;
} pyramid_data_model_agra_PO_AirTargetVolumeCommandType_c;

typedef struct pyramid_data_model_agra_PointingType_Volume_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_PointingType_Volume_List_c;

typedef struct pyramid_data_model_agra_LOS_VariableA_Type_c {
  int32_t reference;
  double azimuth;
  double elevation;
} pyramid_data_model_agra_LOS_VariableA_Type_c;

typedef struct pyramid_data_model_agra_LOS_RatesType_c {
  double azimuth_rate;
  double elevation_rate;
  uint8_t has_roll_rate;
  double roll_rate;
} pyramid_data_model_agra_LOS_RatesType_c;

typedef struct pyramid_data_model_agra_LOS_VariableB_Type_c {
  int32_t reference;  /* from LOS_VariableA_Type */
  double azimuth;  /* from LOS_VariableA_Type */
  double elevation;  /* from LOS_VariableA_Type */
  uint8_t has_roll;
  double roll;
  uint8_t has_rates;
  pyramid_data_model_agra_LOS_RatesType_c rates;
} pyramid_data_model_agra_LOS_VariableB_Type_c;

typedef struct pyramid_data_model_agra_LOS_D_Type_c {
  uint8_t has_los;
  pyramid_data_model_agra_LOS_VariableB_Type_c los;
  uint8_t has_los_rates;
  pyramid_data_model_agra_LOS_RatesType_c los_rates;
} pyramid_data_model_agra_LOS_D_Type_c;

typedef struct pyramid_data_model_agra_PointingType_Geospatial_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_PointingType_Geospatial_List_c;

typedef struct pyramid_data_model_agra_PointingType_c {
  uint8_t has_geospatial;
  pyramid_data_model_agra_PointingType_Geospatial_List_c geospatial;
  uint8_t has_los_option;
  pyramid_data_model_agra_LOS_D_Type_c los_option;
  uint8_t has_volume;
  pyramid_data_model_agra_PointingType_Volume_List_c volume;
  uint8_t has_turret_slaved;
  pyramid_str_t turret_slaved;
  uint8_t has_activity_slaved_id;
  pyramid_data_model_agra_ActivityID_Type_c activity_slaved_id;
  uint8_t has_fixed_pointing;
  int32_t fixed_pointing;
} pyramid_data_model_agra_PointingType_c;

typedef struct pyramid_data_model_agra_PO_TaskType_c {
  int32_t capability;  /* from PO_TaskBaseType */
  pyramid_slice_t sensor_spectrum;  /* from PO_TaskBaseType */
  uint8_t has_look_at_coords;
  pyramid_data_model_agra_LocatedEllipseType_c look_at_coords;  /* from PO_TaskBaseType */
  pyramid_str_t minimum_niirs;  /* from PO_TaskBaseType */
  pyramid_str_t desired_niirs;  /* from PO_TaskBaseType */
  uint8_t has_collection_constraints;
  pyramid_data_model_agra_OpticalCollectionConstraintsType_c collection_constraints;  /* from PO_TaskBaseType */
  uint8_t has_pair_identifier;
  int32_t pair_identifier;  /* from PO_TaskBaseType */
  pyramid_slice_t output;  /* from PO_TaskBaseType */
  pyramid_data_model_agra_PointingType_c target;
} pyramid_data_model_agra_PO_TaskType_c;

typedef struct pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c;

typedef struct pyramid_data_model_agra_MA_JettisonStoreSelectionType_c {
  uint8_t has_jettison_all_capable_weapons;
  pyramid_str_t jettison_all_capable_weapons;
  uint8_t has_capability_id;
  pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c capability_id;
} pyramid_data_model_agra_MA_JettisonStoreSelectionType_c;

typedef struct pyramid_data_model_agra_MA_JettisonTaskType_c {
  pyramid_data_model_agra_MA_JettisonStoreSelectionType_c jettison_store_selection;
  int32_t store_jettison_type;
} pyramid_data_model_agra_MA_JettisonTaskType_c;

typedef struct pyramid_data_model_agra_MA_TaskType_c {
  uint8_t has_air_sample;
  pyramid_data_model_agra_AirSampleTaskType_c air_sample;
  uint8_t has_amti;
  pyramid_data_model_agra_AMTI_TaskType_c amti;
  uint8_t has_ao;
  pyramid_data_model_agra_AO_TaskType_c ao;
  uint8_t has_cap;
  pyramid_data_model_agra_MA_CAP_TaskType_c cap;
  uint8_t has_cargo_delivery;
  pyramid_data_model_agra_CargoDeliveryTaskType_c cargo_delivery;
  uint8_t has_comint;
  pyramid_data_model_agra_COMINT_TaskType_c comint;
  uint8_t has_comm_relay;
  pyramid_data_model_agra_CommRelayTaskType_c comm_relay;
  uint8_t has_counter_space;
  pyramid_data_model_agra_CounterSpaceTaskType_c counter_space;
  uint8_t has_escort;
  pyramid_data_model_agra_MA_EscortTaskType_c escort;
  uint8_t has_ea;
  pyramid_data_model_agra_EA_TaskType_c ea;
  uint8_t has_esm;
  pyramid_data_model_agra_ESM_TaskType_c esm;
  uint8_t has_flight;
  pyramid_data_model_agra_MA_FlightTaskType_c flight;
  uint8_t has_jettison;
  pyramid_data_model_agra_MA_JettisonTaskType_c jettison;
  uint8_t has_orbit_change;
  pyramid_data_model_agra_OrbitChangeTaskType_c orbit_change;
  uint8_t has_orbital_surveillance;
  pyramid_data_model_agra_OrbitalSurveillanceTaskType_c orbital_surveillance;
  uint8_t has_orbital_surveillance_sensor;
  pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c orbital_surveillance_sensor;
  uint8_t has_po;
  pyramid_data_model_agra_PO_TaskType_c po;
  uint8_t has_refuel;
  pyramid_data_model_agra_RefuelTaskType_c refuel;
  uint8_t has_sar;
  pyramid_data_model_agra_SAR_TaskType_c sar;
  uint8_t has_smti;
  pyramid_data_model_agra_SMTI_TaskType_c smti;
  uint8_t has_strike;
  pyramid_data_model_agra_StrikeTaskType_c strike;
  uint8_t has_system_deployment;
  pyramid_data_model_agra_SystemDeploymentTaskType_c system_deployment;
  uint8_t has_tactical_order;
  pyramid_data_model_agra_TacticalOrderTaskType_c tactical_order;
  uint8_t has_weather_radar;
  pyramid_data_model_agra_WeatherRadarTaskType_c weather_radar;
} pyramid_data_model_agra_MA_TaskType_c;

typedef struct pyramid_data_model_agra_MA_ConstraintID_Type_c {
  pyramid_str_t uuid;  /* from ID_Type */
  pyramid_str_t descriptive_label;  /* from ID_Type */
} pyramid_data_model_agra_MA_ConstraintID_Type_c;

typedef struct pyramid_data_model_agra_MA_TaskMDT_c {
  pyramid_data_model_agra_TaskID_Type_c task_id;
  pyramid_data_model_agra_MA_TaskType_c task_type;
  int32_t task_plurality;
  pyramid_slice_t constraint_id;
  uint8_t has_task_constraints;
  pyramid_data_model_agra_MA_RequirementConstraintsType_c task_constraints;
  uint8_t has_task_guidance;
  pyramid_data_model_agra_RequirementGuidanceType_c task_guidance;
  uint8_t has_metadata;
  pyramid_data_model_agra_RequirementMetadataType_c metadata;
} pyramid_data_model_agra_MA_TaskMDT_c;

typedef struct pyramid_data_model_agra_MA_TaskMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MA_TaskMDT_c message_data;
} pyramid_data_model_agra_MA_TaskMT_c;

typedef struct pyramid_data_model_agra_MA_TaskStatusMDT_c {
  pyramid_data_model_agra_MA_PackageSystemType_c executing_system_or_package;  /* from MA_RequirementExecutionStatusDetailsType */
  int32_t execution_state;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_execution_state_reason;
  pyramid_data_model_agra_CannotComplyType_c execution_state_reason;  /* from MA_RequirementExecutionStatusDetailsType */
  pyramid_slice_t actual_timing;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_percent_completed;
  double percent_completed;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_traceability;
  pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c traceability;  /* from MA_RequirementExecutionStatusDetailsType */
  uint8_t has_metrics;
  pyramid_data_model_agra_MetricsType_c metrics;  /* from MA_RequirementExecutionStatusDetailsType */
  pyramid_data_model_agra_TaskID_Type_c task_id;
} pyramid_data_model_agra_MA_TaskStatusMDT_c;

typedef struct pyramid_data_model_agra_MA_TaskStatusMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  pyramid_data_model_agra_MA_TaskStatusMDT_c message_data;
} pyramid_data_model_agra_MA_TaskStatusMT_c;

typedef struct pyramid_data_model_agra_RoutePlanSegmentReferenceType_c {
  pyramid_data_model_agra_RoutePlanID_Type_c route_plan_id;
  pyramid_data_model_agra_PathID_Type_c path_id;
  pyramid_data_model_agra_SegmentID_Type_c path_segment_id;
  uint8_t has_mission_plan_id;
  pyramid_data_model_agra_MissionPlanID_Type_c mission_plan_id;
} pyramid_data_model_agra_RoutePlanSegmentReferenceType_c;

typedef struct pyramid_data_model_agra_ConflictLocationType_c {
  pyramid_data_model_agra_Point2D_Type_c conflict_point;
  uint8_t has_conflicted_route_segment;
  pyramid_data_model_agra_RoutePlanSegmentReferenceType_c conflicted_route_segment;
} pyramid_data_model_agra_ConflictLocationType_c;

typedef struct pyramid_data_model_agra_RequirementsReferenceLockableType_c {
  pyramid_slice_t effect_id;
  pyramid_slice_t action_id;
  pyramid_slice_t task_id;
  pyramid_slice_t response_id;
} pyramid_data_model_agra_RequirementsReferenceLockableType_c;

typedef struct pyramid_data_model_agra_RequirementsReferenceType_c {
  pyramid_slice_t effect_id;  /* from RequirementsReferenceLockableType */
  pyramid_slice_t action_id;  /* from RequirementsReferenceLockableType */
  pyramid_slice_t task_id;  /* from RequirementsReferenceLockableType */
  pyramid_slice_t response_id;  /* from RequirementsReferenceLockableType */
  pyramid_slice_t capability_command_id;
} pyramid_data_model_agra_RequirementsReferenceType_c;

typedef struct pyramid_data_model_agra_ConflictType_c {
  pyramid_data_model_agra_ID_Type_c conflict_id;
  uint8_t has_conflicting_object_id;
  pyramid_data_model_agra_ID_Type_c conflicting_object_id;
  uint8_t has_conflict_location;
  pyramid_data_model_agra_ConflictLocationType_c conflict_location;
  uint8_t has_conflict_time;
  pyramid_data_model_agra_DateTimeRangeType_c conflict_time;
  uint8_t has_affected_requirements;
  pyramid_data_model_agra_RequirementsReferenceType_c affected_requirements;
} pyramid_data_model_agra_ConflictType_c;

typedef struct pyramid_data_model_agra_AutonomousPlanningActionStatusType_c {
  pyramid_data_model_agra_AutonomousPlanningActionID_Type_c autonomous_planning_action_id;
  int32_t status;
} pyramid_data_model_agra_AutonomousPlanningActionStatusType_c;

typedef struct pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c {
  pyramid_slice_t items;
} pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c;

typedef struct pyramid_data_model_agra_AutonomousActionStatusChoiceType_c {
  uint8_t has_autonomous_planning_action_status;
  pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c autonomous_planning_action_status;
  uint8_t has_alert_only;
  pyramid_str_t alert_only;
} pyramid_data_model_agra_AutonomousActionStatusChoiceType_c;

typedef struct pyramid_data_model_agra_MissionContingencyConditionType_c {
  pyramid_data_model_agra_SystemID_Type_c conflicted_system_id;
  int32_t conflict_state;
  pyramid_slice_t conflict;
  uint8_t has_trigger;
  pyramid_data_model_agra_PlanningTriggerType_c trigger;
  uint8_t has_autonomous_action_status;
  pyramid_data_model_agra_AutonomousActionStatusChoiceType_c autonomous_action_status;
  pyramid_slice_t operator_recommendation;
} pyramid_data_model_agra_MissionContingencyConditionType_c;

typedef struct pyramid_data_model_agra_MissionContingencyAlertMDT_c {
  pyramid_data_model_agra_MissionContingencyAlertID_Type_c mission_contingency_alert_id;
  pyramid_data_model_agra_SystemID_Type_c source_system_id;
  pyramid_slice_t contingency_condition;
} pyramid_data_model_agra_MissionContingencyAlertMDT_c;

typedef struct pyramid_data_model_agra_MissionContingencyAlertMT_c {
  pyramid_data_model_agra_SecurityInformationType_c security_information;  /* from MessageType */
  pyramid_data_model_agra_HeaderType_c message_header;  /* from MessageType */
  uint8_t has_object_state;
  int32_t object_state;
  pyramid_data_model_agra_MissionContingencyAlertMDT_c message_data;
} pyramid_data_model_agra_MissionContingencyAlertMT_c;

void pyramid_data_model_agra_MA_ActionMT_c_free(pyramid_data_model_agra_MA_ActionMT_c* value);
void pyramid_data_model_agra_MessageType_c_free(pyramid_data_model_agra_MessageType_c* value);
void pyramid_data_model_agra_SecurityInformationType_c_free(pyramid_data_model_agra_SecurityInformationType_c* value);
void pyramid_data_model_agra_OwnerProducerChoiceType_c_free(pyramid_data_model_agra_OwnerProducerChoiceType_c* value);
void pyramid_data_model_agra_SCI_ControlsChoiceType_c_free(pyramid_data_model_agra_SCI_ControlsChoiceType_c* value);
void pyramid_data_model_agra_AtomicEnergyMarkingsChoiceType_c_free(pyramid_data_model_agra_AtomicEnergyMarkingsChoiceType_c* value);
void pyramid_data_model_agra_ReleasableToChoiceType_c_free(pyramid_data_model_agra_ReleasableToChoiceType_c* value);
void pyramid_data_model_agra_FGI_SourceOpenChoiceType_c_free(pyramid_data_model_agra_FGI_SourceOpenChoiceType_c* value);
void pyramid_data_model_agra_NonIC_MarkingsChoiceType_c_free(pyramid_data_model_agra_NonIC_MarkingsChoiceType_c* value);
void pyramid_data_model_agra_HeaderType_c_free(pyramid_data_model_agra_HeaderType_c* value);
void pyramid_data_model_agra_SystemID_Type_c_free(pyramid_data_model_agra_SystemID_Type_c* value);
void pyramid_data_model_agra_ID_Type_c_free(pyramid_data_model_agra_ID_Type_c* value);
void pyramid_data_model_agra_ServiceID_Type_c_free(pyramid_data_model_agra_ServiceID_Type_c* value);
void pyramid_data_model_agra_MissionID_Type_c_free(pyramid_data_model_agra_MissionID_Type_c* value);
void pyramid_data_model_agra_VersionedID_Type_c_free(pyramid_data_model_agra_VersionedID_Type_c* value);
void pyramid_data_model_agra_MA_ActionMDT_c_free(pyramid_data_model_agra_MA_ActionMDT_c* value);
void pyramid_data_model_agra_ActionID_Type_c_free(pyramid_data_model_agra_ActionID_Type_c* value);
void pyramid_data_model_agra_MA_RequirementConstraintsType_c_free(pyramid_data_model_agra_MA_RequirementConstraintsType_c* value);
void pyramid_data_model_agra_ComparableRankingType_c_free(pyramid_data_model_agra_ComparableRankingType_c* value);
void pyramid_data_model_agra_MA_RequirementAllocationParametersType_c_free(pyramid_data_model_agra_MA_RequirementAllocationParametersType_c* value);
void pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c_free(pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c* value);
void pyramid_data_model_agra_MA_AllocationChoiceType_c_free(pyramid_data_model_agra_MA_AllocationChoiceType_c* value);
void pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c_free(pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c* value);
void pyramid_data_model_agra_PackageID_Type_c_free(pyramid_data_model_agra_PackageID_Type_c* value);
void pyramid_data_model_agra_IdentityType_c_free(pyramid_data_model_agra_IdentityType_c* value);
void pyramid_data_model_agra_StandardIdentityType_c_free(pyramid_data_model_agra_StandardIdentityType_c* value);
void pyramid_data_model_agra_CountryCodeType_c_free(pyramid_data_model_agra_CountryCodeType_c* value);
void pyramid_data_model_agra_EnvironmentIdentityType_c_free(pyramid_data_model_agra_EnvironmentIdentityType_c* value);
void pyramid_data_model_agra_OrbitRegimeType_c_free(pyramid_data_model_agra_OrbitRegimeType_c* value);
void pyramid_data_model_agra_PlatformIdentityType_c_free(pyramid_data_model_agra_PlatformIdentityType_c* value);
void pyramid_data_model_agra_SpecificIdentityType_c_free(pyramid_data_model_agra_SpecificIdentityType_c* value);
void pyramid_data_model_agra_EmitterIdentityType_c_free(pyramid_data_model_agra_EmitterIdentityType_c* value);
void pyramid_data_model_agra_EmitterIdentityCategoryType_c_free(pyramid_data_model_agra_EmitterIdentityCategoryType_c* value);
void pyramid_data_model_agra_RadarEmitterIdentityType_c_free(pyramid_data_model_agra_RadarEmitterIdentityType_c* value);
void pyramid_data_model_agra_ForeignKeyType_c_free(pyramid_data_model_agra_ForeignKeyType_c* value);
void pyramid_data_model_agra_CommunicationsEmitterIdentityType_c_free(pyramid_data_model_agra_CommunicationsEmitterIdentityType_c* value);
void pyramid_data_model_agra_JammerEmitterIdentityType_c_free(pyramid_data_model_agra_JammerEmitterIdentityType_c* value);
void pyramid_data_model_agra_MissileEmitterIdentityType_c_free(pyramid_data_model_agra_MissileEmitterIdentityType_c* value);
void pyramid_data_model_agra_SpecificEmitterIdentityType_c_free(pyramid_data_model_agra_SpecificEmitterIdentityType_c* value);
void pyramid_data_model_agra_FacilityIdentificationType_c_free(pyramid_data_model_agra_FacilityIdentificationType_c* value);
void pyramid_data_model_agra_VehicleIdentificationType_c_free(pyramid_data_model_agra_VehicleIdentificationType_c* value);
void pyramid_data_model_agra_VehicleUniqueIdentifierType_c_free(pyramid_data_model_agra_VehicleUniqueIdentifierType_c* value);
void pyramid_data_model_agra_AIS_Type_c_free(pyramid_data_model_agra_AIS_Type_c* value);
void pyramid_data_model_agra_SatelliteIdentifierType_c_free(pyramid_data_model_agra_SatelliteIdentifierType_c* value);
void pyramid_data_model_agra_InternationalDesignatorType_c_free(pyramid_data_model_agra_InternationalDesignatorType_c* value);
void pyramid_data_model_agra_MissionInformationType_c_free(pyramid_data_model_agra_MissionInformationType_c* value);
void pyramid_data_model_agra_IFF_Type_c_free(pyramid_data_model_agra_IFF_Type_c* value);
void pyramid_data_model_agra_IFF_Mode1Type_c_free(pyramid_data_model_agra_IFF_Mode1Type_c* value);
void pyramid_data_model_agra_IFF_OctalModeType_c_free(pyramid_data_model_agra_IFF_OctalModeType_c* value);
void pyramid_data_model_agra_IFF_Mode4Type_c_free(pyramid_data_model_agra_IFF_Mode4Type_c* value);
void pyramid_data_model_agra_IFF_Mode5Type_c_free(pyramid_data_model_agra_IFF_Mode5Type_c* value);
void pyramid_data_model_agra_IFF_ModeS_Type_c_free(pyramid_data_model_agra_IFF_ModeS_Type_c* value);
void pyramid_data_model_agra_CallSignType_c_free(pyramid_data_model_agra_CallSignType_c* value);
void pyramid_data_model_agra_DataLinkIdentifierPET_c_free(pyramid_data_model_agra_DataLinkIdentifierPET_c* value);
void pyramid_data_model_agra_EOB_IdentityType_c_free(pyramid_data_model_agra_EOB_IdentityType_c* value);
void pyramid_data_model_agra_EOB_SiteIdentityType_c_free(pyramid_data_model_agra_EOB_SiteIdentityType_c* value);
void pyramid_data_model_agra_EOB_IdentityBaseType_c_free(pyramid_data_model_agra_EOB_IdentityBaseType_c* value);
void pyramid_data_model_agra_EOB_SitePIN_Type_c_free(pyramid_data_model_agra_EOB_SitePIN_Type_c* value);
void pyramid_data_model_agra_BasicEncyclopediaNumberType_c_free(pyramid_data_model_agra_BasicEncyclopediaNumberType_c* value);
void pyramid_data_model_agra_EOB_EquipmentIdentityType_c_free(pyramid_data_model_agra_EOB_EquipmentIdentityType_c* value);
void pyramid_data_model_agra_EOB_EquipmentType_c_free(pyramid_data_model_agra_EOB_EquipmentType_c* value);
void pyramid_data_model_agra_StoreType_c_free(pyramid_data_model_agra_StoreType_c* value);
void pyramid_data_model_agra_CapabilityID_Type_c_free(pyramid_data_model_agra_CapabilityID_Type_c* value);
void pyramid_data_model_agra_RequirementTimingType_c_free(pyramid_data_model_agra_RequirementTimingType_c* value);
void pyramid_data_model_agra_TimingConstraintsType_c_free(pyramid_data_model_agra_TimingConstraintsType_c* value);
void pyramid_data_model_agra_TimeWindowType_c_free(pyramid_data_model_agra_TimeWindowType_c* value);
void pyramid_data_model_agra_DateTimeRangeType_c_free(pyramid_data_model_agra_DateTimeRangeType_c* value);
void pyramid_data_model_agra_WeekdayIntervalType_c_free(pyramid_data_model_agra_WeekdayIntervalType_c* value);
void pyramid_data_model_agra_RepetitionConstraintsType_c_free(pyramid_data_model_agra_RepetitionConstraintsType_c* value);
void pyramid_data_model_agra_RepetitionType_c_free(pyramid_data_model_agra_RepetitionType_c* value);
void pyramid_data_model_agra_RepetitionTimeBasedType_c_free(pyramid_data_model_agra_RepetitionTimeBasedType_c* value);
void pyramid_data_model_agra_RepetitionContinuousType_c_free(pyramid_data_model_agra_RepetitionContinuousType_c* value);
void pyramid_data_model_agra_RepetitionFiniteType_c_free(pyramid_data_model_agra_RepetitionFiniteType_c* value);
void pyramid_data_model_agra_RepetitionPeriodicType_c_free(pyramid_data_model_agra_RepetitionPeriodicType_c* value);
void pyramid_data_model_agra_RepetitionEventBasedType_c_free(pyramid_data_model_agra_RepetitionEventBasedType_c* value);
void pyramid_data_model_agra_RepetitionEventType_c_free(pyramid_data_model_agra_RepetitionEventType_c* value);
void pyramid_data_model_agra_RepetitionPositionChangeType_c_free(pyramid_data_model_agra_RepetitionPositionChangeType_c* value);
void pyramid_data_model_agra_LOS_Type_c_free(pyramid_data_model_agra_LOS_Type_c* value);
void pyramid_data_model_agra_LOS_InertialA_Type_c_free(pyramid_data_model_agra_LOS_InertialA_Type_c* value);
void pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c_free(pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c* value);
void pyramid_data_model_agra_RTN_PositionDeltaType_c_free(pyramid_data_model_agra_RTN_PositionDeltaType_c* value);
void pyramid_data_model_agra_RTN_VelocityDeltaType_c_free(pyramid_data_model_agra_RTN_VelocityDeltaType_c* value);
void pyramid_data_model_agra_TimeErrorType_c_free(pyramid_data_model_agra_TimeErrorType_c* value);
void pyramid_data_model_agra_EventOffsetChoiceType_c_free(pyramid_data_model_agra_EventOffsetChoiceType_c* value);
void pyramid_data_model_agra_LOS_InertialB_Type_c_free(pyramid_data_model_agra_LOS_InertialB_Type_c* value);
void pyramid_data_model_agra_EventWindowChoiceType_c_free(pyramid_data_model_agra_EventWindowChoiceType_c* value);
void pyramid_data_model_agra_ReferenceAssetKinematicsType_c_free(pyramid_data_model_agra_ReferenceAssetKinematicsType_c* value);
void pyramid_data_model_agra_AssetType_c_free(pyramid_data_model_agra_AssetType_c* value);
void pyramid_data_model_agra_EntityID_Type_c_free(pyramid_data_model_agra_EntityID_Type_c* value);
void pyramid_data_model_agra_OrbitKinematicsType_c_free(pyramid_data_model_agra_OrbitKinematicsType_c* value);
void pyramid_data_model_agra_TLE_BaseType_c_free(pyramid_data_model_agra_TLE_BaseType_c* value);
void pyramid_data_model_agra_COE_OrbitBaseType_c_free(pyramid_data_model_agra_COE_OrbitBaseType_c* value);
void pyramid_data_model_agra_COE_OrientationType_c_free(pyramid_data_model_agra_COE_OrientationType_c* value);
void pyramid_data_model_agra_COE_NonEquatorialOrientationType_c_free(pyramid_data_model_agra_COE_NonEquatorialOrientationType_c* value);
void pyramid_data_model_agra_COE_EquatorialOrientationType_c_free(pyramid_data_model_agra_COE_EquatorialOrientationType_c* value);
void pyramid_data_model_agra_COE_PositionType_c_free(pyramid_data_model_agra_COE_PositionType_c* value);
void pyramid_data_model_agra_OrbitalEphemerisChoiceType_c_free(pyramid_data_model_agra_OrbitalEphemerisChoiceType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c_free(pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c* value);
void pyramid_data_model_agra_J2K_KinematicsType_c_free(pyramid_data_model_agra_J2K_KinematicsType_c* value);
void pyramid_data_model_agra_J2K_PositionType_c_free(pyramid_data_model_agra_J2K_PositionType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsVelocityType_c_free(pyramid_data_model_agra_OrbitalKinematicsVelocityType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c_free(pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c* value);
void pyramid_data_model_agra_QuaternionType_c_free(pyramid_data_model_agra_QuaternionType_c* value);
void pyramid_data_model_agra_CovarianceMatrixType_c_free(pyramid_data_model_agra_CovarianceMatrixType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c_free(pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c* value);
void pyramid_data_model_agra_GCRS_KinematicsType_c_free(pyramid_data_model_agra_GCRS_KinematicsType_c* value);
void pyramid_data_model_agra_GCRS_PositionType_c_free(pyramid_data_model_agra_GCRS_PositionType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c_free(pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c* value);
void pyramid_data_model_agra_BCRS_KinematicsType_c_free(pyramid_data_model_agra_BCRS_KinematicsType_c* value);
void pyramid_data_model_agra_BCRS_PositionType_c_free(pyramid_data_model_agra_BCRS_PositionType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c_free(pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c* value);
void pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c_free(pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsRelativeStateVectorType_c_free(pyramid_data_model_agra_OrbitalKinematicsRelativeStateVectorType_c* value);
void pyramid_data_model_agra_RTN_KinematicsType_c_free(pyramid_data_model_agra_RTN_KinematicsType_c* value);
void pyramid_data_model_agra_RTN_PositionType_c_free(pyramid_data_model_agra_RTN_PositionType_c* value);
void pyramid_data_model_agra_RTN_VelocityType_c_free(pyramid_data_model_agra_RTN_VelocityType_c* value);
void pyramid_data_model_agra_RTN_AccelerationType_c_free(pyramid_data_model_agra_RTN_AccelerationType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c_free(pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c* value);
void pyramid_data_model_agra_OrbitalVCM_Type_c_free(pyramid_data_model_agra_OrbitalVCM_Type_c* value);
void pyramid_data_model_agra_VCM_DataType_c_free(pyramid_data_model_agra_VCM_DataType_c* value);
void pyramid_data_model_agra_RTN_PositionSigmaType_c_free(pyramid_data_model_agra_RTN_PositionSigmaType_c* value);
void pyramid_data_model_agra_RTN_VelocitySigmaType_c_free(pyramid_data_model_agra_RTN_VelocitySigmaType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsChoiceType_c_free(pyramid_data_model_agra_OrbitalKinematicsChoiceType_c* value);
void pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c_free(pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c* value);
void pyramid_data_model_agra_MA_RequirementKinematicConstraintsType_c_free(pyramid_data_model_agra_MA_RequirementKinematicConstraintsType_c* value);
void pyramid_data_model_agra_GeoFiltersQueryType_c_free(pyramid_data_model_agra_GeoFiltersQueryType_c* value);
void pyramid_data_model_agra_QueryPET_c_free(pyramid_data_model_agra_QueryPET_c* value);
void pyramid_data_model_agra_ZoneType_c_free(pyramid_data_model_agra_ZoneType_c* value);
void pyramid_data_model_agra_AreaChoiceType_c_free(pyramid_data_model_agra_AreaChoiceType_c* value);
void pyramid_data_model_agra_PolygonType_c_free(pyramid_data_model_agra_PolygonType_c* value);
void pyramid_data_model_agra_PolygonPointChoiceType_c_free(pyramid_data_model_agra_PolygonPointChoiceType_c* value);
void pyramid_data_model_agra_Point2D_Type_c_free(pyramid_data_model_agra_Point2D_Type_c* value);
void pyramid_data_model_agra_AltitudeRangeType_c_free(pyramid_data_model_agra_AltitudeRangeType_c* value);
void pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c_free(pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c* value);
void pyramid_data_model_agra_PolygonRelativeType_c_free(pyramid_data_model_agra_PolygonRelativeType_c* value);
void pyramid_data_model_agra_ReferenceFrameID_Type_c_free(pyramid_data_model_agra_ReferenceFrameID_Type_c* value);
void pyramid_data_model_agra_RelativeOffset2D_Type_c_free(pyramid_data_model_agra_RelativeOffset2D_Type_c* value);
void pyramid_data_model_agra_Z_ChoiceType_c_free(pyramid_data_model_agra_Z_ChoiceType_c* value);
void pyramid_data_model_agra_AltitudeOffsetReferenceType_c_free(pyramid_data_model_agra_AltitudeOffsetReferenceType_c* value);
void pyramid_data_model_agra_AltitudeReferenceType_c_free(pyramid_data_model_agra_AltitudeReferenceType_c* value);
void pyramid_data_model_agra_LocatedEllipseType_c_free(pyramid_data_model_agra_LocatedEllipseType_c* value);
void pyramid_data_model_agra_EllipseType_c_free(pyramid_data_model_agra_EllipseType_c* value);
void pyramid_data_model_agra_PointChoiceType_c_free(pyramid_data_model_agra_PointChoiceType_c* value);
void pyramid_data_model_agra_Point2D_RelativeType_c_free(pyramid_data_model_agra_Point2D_RelativeType_c* value);
void pyramid_data_model_agra_LocatedRectangleType_c_free(pyramid_data_model_agra_LocatedRectangleType_c* value);
void pyramid_data_model_agra_RectangleType_c_free(pyramid_data_model_agra_RectangleType_c* value);
void pyramid_data_model_agra_SlantRangeAreaType_c_free(pyramid_data_model_agra_SlantRangeAreaType_c* value);
void pyramid_data_model_agra_SlantRangeConstraintsType_c_free(pyramid_data_model_agra_SlantRangeConstraintsType_c* value);
void pyramid_data_model_agra_AnglePairType_c_free(pyramid_data_model_agra_AnglePairType_c* value);
void pyramid_data_model_agra_OrbitalFiltersQueryType_c_free(pyramid_data_model_agra_OrbitalFiltersQueryType_c* value);
void pyramid_data_model_agra_OpVolumeType_c_free(pyramid_data_model_agra_OpVolumeType_c* value);
void pyramid_data_model_agra_GeometricVolumeType_c_free(pyramid_data_model_agra_GeometricVolumeType_c* value);
void pyramid_data_model_agra_Shape3D_ChoiceType_c_free(pyramid_data_model_agra_Shape3D_ChoiceType_c* value);
void pyramid_data_model_agra_SphereType_c_free(pyramid_data_model_agra_SphereType_c* value);
void pyramid_data_model_agra_DomeType_c_free(pyramid_data_model_agra_DomeType_c* value);
void pyramid_data_model_agra_EllipsoidType_c_free(pyramid_data_model_agra_EllipsoidType_c* value);
void pyramid_data_model_agra_CylinderType_c_free(pyramid_data_model_agra_CylinderType_c* value);
void pyramid_data_model_agra_ConeType_c_free(pyramid_data_model_agra_ConeType_c* value);
void pyramid_data_model_agra_RectangularConeType_c_free(pyramid_data_model_agra_RectangularConeType_c* value);
void pyramid_data_model_agra_ArcVolumeType_c_free(pyramid_data_model_agra_ArcVolumeType_c* value);
void pyramid_data_model_agra_AlongOrbitalArcDeltaType_c_free(pyramid_data_model_agra_AlongOrbitalArcDeltaType_c* value);
void pyramid_data_model_agra_IncRaPeriodVolumeType_c_free(pyramid_data_model_agra_IncRaPeriodVolumeType_c* value);
void pyramid_data_model_agra_KinematicsChoiceType_c_free(pyramid_data_model_agra_KinematicsChoiceType_c* value);
void pyramid_data_model_agra_OpVolumeKinematicsType_c_free(pyramid_data_model_agra_OpVolumeKinematicsType_c* value);
void pyramid_data_model_agra_Velocity2D_Type_c_free(pyramid_data_model_agra_Velocity2D_Type_c* value);
void pyramid_data_model_agra_RTN_LocalPositionType_c_free(pyramid_data_model_agra_RTN_LocalPositionType_c* value);
void pyramid_data_model_agra_KinematicsOptionsType_c_free(pyramid_data_model_agra_KinematicsOptionsType_c* value);
void pyramid_data_model_agra_KinematicsMultiStandardType_c_free(pyramid_data_model_agra_KinematicsMultiStandardType_c* value);
void pyramid_data_model_agra_KinematicsType_c_free(pyramid_data_model_agra_KinematicsType_c* value);
void pyramid_data_model_agra_EntityPositionType_c_free(pyramid_data_model_agra_EntityPositionType_c* value);
void pyramid_data_model_agra_FixedPositionType_c_free(pyramid_data_model_agra_FixedPositionType_c* value);
void pyramid_data_model_agra_Point2D_ReportedType_c_free(pyramid_data_model_agra_Point2D_ReportedType_c* value);
void pyramid_data_model_agra_UncertaintyType_c_free(pyramid_data_model_agra_UncertaintyType_c* value);
void pyramid_data_model_agra_RelativePositionType_c_free(pyramid_data_model_agra_RelativePositionType_c* value);
void pyramid_data_model_agra_Point2D_ReferenceType_c_free(pyramid_data_model_agra_Point2D_ReferenceType_c* value);
void pyramid_data_model_agra_ReferenceObjectType_c_free(pyramid_data_model_agra_ReferenceObjectType_c* value);
void pyramid_data_model_agra_OpPointID_Type_c_free(pyramid_data_model_agra_OpPointID_Type_c* value);
void pyramid_data_model_agra_LineOfSightChoiceType_c_free(pyramid_data_model_agra_LineOfSightChoiceType_c* value);
void pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c_free(pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c* value);
void pyramid_data_model_agra_LOS_MeasurementType_c_free(pyramid_data_model_agra_LOS_MeasurementType_c* value);
void pyramid_data_model_agra_LOS_AzElType_c_free(pyramid_data_model_agra_LOS_AzElType_c* value);
void pyramid_data_model_agra_LOS_AzElRatesType_c_free(pyramid_data_model_agra_LOS_AzElRatesType_c* value);
void pyramid_data_model_agra_ConeAngleType_c_free(pyramid_data_model_agra_ConeAngleType_c* value);
void pyramid_data_model_agra_ConeAngleRatesType_c_free(pyramid_data_model_agra_ConeAngleRatesType_c* value);
void pyramid_data_model_agra_ArrivalDataType_c_free(pyramid_data_model_agra_ArrivalDataType_c* value);
void pyramid_data_model_agra_SlantRangeType_c_free(pyramid_data_model_agra_SlantRangeType_c* value);
void pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c_free(pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c* value);
void pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c_free(pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c* value);
void pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c_free(pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c* value);
void pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c_free(pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c* value);
void pyramid_data_model_agra_LOS_VarianceType_c_free(pyramid_data_model_agra_LOS_VarianceType_c* value);
void pyramid_data_model_agra_LOS_VarianceRatesType_c_free(pyramid_data_model_agra_LOS_VarianceRatesType_c* value);
void pyramid_data_model_agra_LOS_CovarianceType_c_free(pyramid_data_model_agra_LOS_CovarianceType_c* value);
void pyramid_data_model_agra_LOS_CovariancesRatesType_c_free(pyramid_data_model_agra_LOS_CovariancesRatesType_c* value);
void pyramid_data_model_agra_ConeAngleUncertaintyType_c_free(pyramid_data_model_agra_ConeAngleUncertaintyType_c* value);
void pyramid_data_model_agra_ConeAngleVarianceType_c_free(pyramid_data_model_agra_ConeAngleVarianceType_c* value);
void pyramid_data_model_agra_ConeAngleVarianceRatesType_c_free(pyramid_data_model_agra_ConeAngleVarianceRatesType_c* value);
void pyramid_data_model_agra_ConeAngleCovarianceType_c_free(pyramid_data_model_agra_ConeAngleCovarianceType_c* value);
void pyramid_data_model_agra_ConeAngleCovarianceRatesType_c_free(pyramid_data_model_agra_ConeAngleCovarianceRatesType_c* value);
void pyramid_data_model_agra_ArrivalDataUncertaintyType_c_free(pyramid_data_model_agra_ArrivalDataUncertaintyType_c* value);
void pyramid_data_model_agra_ArrivalDataVarianceType_c_free(pyramid_data_model_agra_ArrivalDataVarianceType_c* value);
void pyramid_data_model_agra_SlantRangeUncertaintyType_c_free(pyramid_data_model_agra_SlantRangeUncertaintyType_c* value);
void pyramid_data_model_agra_SlantRangeVarianceType_c_free(pyramid_data_model_agra_SlantRangeVarianceType_c* value);
void pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c_free(pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c* value);
void pyramid_data_model_agra_SlantRangeCovarianceType_c_free(pyramid_data_model_agra_SlantRangeCovarianceType_c* value);
void pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c_free(pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c* value);
void pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c_free(pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c* value);
void pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c_free(pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c* value);
void pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c_free(pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c* value);
void pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c_free(pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c* value);
void pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c_free(pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c* value);
void pyramid_data_model_agra_LOS_UncertaintyType_c_free(pyramid_data_model_agra_LOS_UncertaintyType_c* value);
void pyramid_data_model_agra_LOS3D_KinematicsType_c_free(pyramid_data_model_agra_LOS3D_KinematicsType_c* value);
void pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c_free(pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c* value);
void pyramid_data_model_agra_UnitVectorType_c_free(pyramid_data_model_agra_UnitVectorType_c* value);
void pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c_free(pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c* value);
void pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c_free(pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c* value);
void pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c_free(pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c* value);
void pyramid_data_model_agra_LOS3D_CovarianceType_c_free(pyramid_data_model_agra_LOS3D_CovarianceType_c* value);
void pyramid_data_model_agra_Velocity2D_UncertaintyType_c_free(pyramid_data_model_agra_Velocity2D_UncertaintyType_c* value);
void pyramid_data_model_agra_Acceleration3D_Type_c_free(pyramid_data_model_agra_Acceleration3D_Type_c* value);
void pyramid_data_model_agra_StateCovarianceNED_Type_c_free(pyramid_data_model_agra_StateCovarianceNED_Type_c* value);
void pyramid_data_model_agra_PositionAndVelocityCovarianceType_c_free(pyramid_data_model_agra_PositionAndVelocityCovarianceType_c* value);
void pyramid_data_model_agra_PositionPositionCovarianceType_c_free(pyramid_data_model_agra_PositionPositionCovarianceType_c* value);
void pyramid_data_model_agra_PositionVelocityCovarianceType_c_free(pyramid_data_model_agra_PositionVelocityCovarianceType_c* value);
void pyramid_data_model_agra_VelocityVelocityCovarianceType_c_free(pyramid_data_model_agra_VelocityVelocityCovarianceType_c* value);
void pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c_free(pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c* value);
void pyramid_data_model_agra_PositionAccelerationCovarianceType_c_free(pyramid_data_model_agra_PositionAccelerationCovarianceType_c* value);
void pyramid_data_model_agra_VelocityAccelerationCovarianceType_c_free(pyramid_data_model_agra_VelocityAccelerationCovarianceType_c* value);
void pyramid_data_model_agra_OrientationType_c_free(pyramid_data_model_agra_OrientationType_c* value);
void pyramid_data_model_agra_OrientationCovarianceType_c_free(pyramid_data_model_agra_OrientationCovarianceType_c* value);
void pyramid_data_model_agra_AngleHalfPairType_c_free(pyramid_data_model_agra_AngleHalfPairType_c* value);
void pyramid_data_model_agra_GeocentricVolumeType_c_free(pyramid_data_model_agra_GeocentricVolumeType_c* value);
void pyramid_data_model_agra_RequirementDependencyType_c_free(pyramid_data_model_agra_RequirementDependencyType_c* value);
void pyramid_data_model_agra_RequirementDependencyBaseType_c_free(pyramid_data_model_agra_RequirementDependencyBaseType_c* value);
void pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c_free(pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c* value);
void pyramid_data_model_agra_EffectID_Type_c_free(pyramid_data_model_agra_EffectID_Type_c* value);
void pyramid_data_model_agra_TaskID_Type_c_free(pyramid_data_model_agra_TaskID_Type_c* value);
void pyramid_data_model_agra_CommandID_Type_c_free(pyramid_data_model_agra_CommandID_Type_c* value);
void pyramid_data_model_agra_ResponseID_Type_c_free(pyramid_data_model_agra_ResponseID_Type_c* value);
void pyramid_data_model_agra_DataProductClassificationLevelType_c_free(pyramid_data_model_agra_DataProductClassificationLevelType_c* value);
void pyramid_data_model_agra_MA_AnalyticConstraintsType_c_free(pyramid_data_model_agra_MA_AnalyticConstraintsType_c* value);
void pyramid_data_model_agra_OpConstraintWeightingType_c_free(pyramid_data_model_agra_OpConstraintWeightingType_c* value);
void pyramid_data_model_agra_OpConstraintWeightingValueType_c_free(pyramid_data_model_agra_OpConstraintWeightingValueType_c* value);
void pyramid_data_model_agra_PercentRangeType_c_free(pyramid_data_model_agra_PercentRangeType_c* value);
void pyramid_data_model_agra_FileLocationID_Type_c_free(pyramid_data_model_agra_FileLocationID_Type_c* value);
void pyramid_data_model_agra_SurvivabilityRiskSettingType_c_free(pyramid_data_model_agra_SurvivabilityRiskSettingType_c* value);
void pyramid_data_model_agra_RiskSettingType_c_free(pyramid_data_model_agra_RiskSettingType_c* value);
void pyramid_data_model_agra_VulnerabilityLevelsType_c_free(pyramid_data_model_agra_VulnerabilityLevelsType_c* value);
void pyramid_data_model_agra_MA_AccessAssessmentFilterType_c_free(pyramid_data_model_agra_MA_AccessAssessmentFilterType_c* value);
void pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c_free(pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c* value);
void pyramid_data_model_agra_CapabilityTaxonomyUniversalBaseType_c_free(pyramid_data_model_agra_CapabilityTaxonomyUniversalBaseType_c* value);
void pyramid_data_model_agra_CapabilityTaxonomyType_c_free(pyramid_data_model_agra_CapabilityTaxonomyType_c* value);
void pyramid_data_model_agra_AMTI_SpecificDataType_c_free(pyramid_data_model_agra_AMTI_SpecificDataType_c* value);
void pyramid_data_model_agra_CargoDeliverySpecificDataType_c_free(pyramid_data_model_agra_CargoDeliverySpecificDataType_c* value);
void pyramid_data_model_agra_COMINT_SpecificDataType_c_free(pyramid_data_model_agra_COMINT_SpecificDataType_c* value);
void pyramid_data_model_agra_ESM_SpecificDataType_c_free(pyramid_data_model_agra_ESM_SpecificDataType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSpecificDataType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSpecificDataType_c* value);
void pyramid_data_model_agra_SAR_SpecificDataType_c_free(pyramid_data_model_agra_SAR_SpecificDataType_c* value);
void pyramid_data_model_agra_SMTI_SpecificDataType_c_free(pyramid_data_model_agra_SMTI_SpecificDataType_c* value);
void pyramid_data_model_agra_MA_AssetFilterType_c_free(pyramid_data_model_agra_MA_AssetFilterType_c* value);
void pyramid_data_model_agra_MA_SystemFilterType_c_free(pyramid_data_model_agra_MA_SystemFilterType_c* value);
void pyramid_data_model_agra_MA_SystemComparativeType_c_free(pyramid_data_model_agra_MA_SystemComparativeType_c* value);
void pyramid_data_model_agra_MA_SystemCharacteristicType_c_free(pyramid_data_model_agra_MA_SystemCharacteristicType_c* value);
void pyramid_data_model_agra_EntityIdentityChoiceType_c_free(pyramid_data_model_agra_EntityIdentityChoiceType_c* value);
void pyramid_data_model_agra_MA_PrioritizationListValueType_c_free(pyramid_data_model_agra_MA_PrioritizationListValueType_c* value);
void pyramid_data_model_agra_MA_PrioritizationType_c_free(pyramid_data_model_agra_MA_PrioritizationType_c* value);
void pyramid_data_model_agra_MA_RequirementTaxonomyType_c_free(pyramid_data_model_agra_MA_RequirementTaxonomyType_c* value);
void pyramid_data_model_agra_BehaviorType_c_free(pyramid_data_model_agra_BehaviorType_c* value);
void pyramid_data_model_agra_ActivityByType_c_free(pyramid_data_model_agra_ActivityByType_c* value);
void pyramid_data_model_agra_MA_EntityFilterType_c_free(pyramid_data_model_agra_MA_EntityFilterType_c* value);
void pyramid_data_model_agra_MA_DesignationFilterType_c_free(pyramid_data_model_agra_MA_DesignationFilterType_c* value);
void pyramid_data_model_agra_MA_EntityComparativeType_c_free(pyramid_data_model_agra_MA_EntityComparativeType_c* value);
void pyramid_data_model_agra_MA_EntityCharacteristicType_c_free(pyramid_data_model_agra_MA_EntityCharacteristicType_c* value);
void pyramid_data_model_agra_IdentityComparisonType_c_free(pyramid_data_model_agra_IdentityComparisonType_c* value);
void pyramid_data_model_agra_AccessEventFilterType_c_free(pyramid_data_model_agra_AccessEventFilterType_c* value);
void pyramid_data_model_agra_DurationRangeType_c_free(pyramid_data_model_agra_DurationRangeType_c* value);
void pyramid_data_model_agra_DistanceConstraintsType_c_free(pyramid_data_model_agra_DistanceConstraintsType_c* value);
void pyramid_data_model_agra_RF_TaskPerformanceType_c_free(pyramid_data_model_agra_RF_TaskPerformanceType_c* value);
void pyramid_data_model_agra_RF_TaskPerformanceConstraintType_c_free(pyramid_data_model_agra_RF_TaskPerformanceConstraintType_c* value);
void pyramid_data_model_agra_PerformanceMetricSetType_c_free(pyramid_data_model_agra_PerformanceMetricSetType_c* value);
void pyramid_data_model_agra_MetricValueType_c_free(pyramid_data_model_agra_MetricValueType_c* value);
void pyramid_data_model_agra_RF_TaskNormalizedMetricsType_c_free(pyramid_data_model_agra_RF_TaskNormalizedMetricsType_c* value);
void pyramid_data_model_agra_NormalizationTableType_c_free(pyramid_data_model_agra_NormalizationTableType_c* value);
void pyramid_data_model_agra_MA_ShotDoctrineParametersType_c_free(pyramid_data_model_agra_MA_ShotDoctrineParametersType_c* value);
void pyramid_data_model_agra_RequirementGuidanceType_c_free(pyramid_data_model_agra_RequirementGuidanceType_c* value);
void pyramid_data_model_agra_MA_UniqueActionDataType_c_free(pyramid_data_model_agra_MA_UniqueActionDataType_c* value);
void pyramid_data_model_agra_MA_AttackKineticParametersType_c_free(pyramid_data_model_agra_MA_AttackKineticParametersType_c* value);
void pyramid_data_model_agra_MA_EngagementParametersType_c_free(pyramid_data_model_agra_MA_EngagementParametersType_c* value);
void pyramid_data_model_agra_MA_InterceptTacticType_c_free(pyramid_data_model_agra_MA_InterceptTacticType_c* value);
void pyramid_data_model_agra_MA_SkateType_c_free(pyramid_data_model_agra_MA_SkateType_c* value);
void pyramid_data_model_agra_MA_BanzaiType_c_free(pyramid_data_model_agra_MA_BanzaiType_c* value);
void pyramid_data_model_agra_IdentityKindInstanceType_c_free(pyramid_data_model_agra_IdentityKindInstanceType_c* value);
void pyramid_data_model_agra_TargetType_c_free(pyramid_data_model_agra_TargetType_c* value);
void pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c_free(pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c* value);
void pyramid_data_model_agra_SignalID_Type_c_free(pyramid_data_model_agra_SignalID_Type_c* value);
void pyramid_data_model_agra_OpZoneID_Type_c_free(pyramid_data_model_agra_OpZoneID_Type_c* value);
void pyramid_data_model_agra_OpVolumeID_Type_c_free(pyramid_data_model_agra_OpVolumeID_Type_c* value);
void pyramid_data_model_agra_OpLineID_Type_c_free(pyramid_data_model_agra_OpLineID_Type_c* value);
void pyramid_data_model_agra_PointTargetType_c_free(pyramid_data_model_agra_PointTargetType_c* value);
void pyramid_data_model_agra_ZoneExternalType_c_free(pyramid_data_model_agra_ZoneExternalType_c* value);
void pyramid_data_model_agra_LineTargetType_c_free(pyramid_data_model_agra_LineTargetType_c* value);
void pyramid_data_model_agra_LineType_c_free(pyramid_data_model_agra_LineType_c* value);
void pyramid_data_model_agra_LinePointChoiceType_c_free(pyramid_data_model_agra_LinePointChoiceType_c* value);
void pyramid_data_model_agra_LinePoint2D_Type_c_free(pyramid_data_model_agra_LinePoint2D_Type_c* value);
void pyramid_data_model_agra_LinePointChoiceType_Point_List_c_free(pyramid_data_model_agra_LinePointChoiceType_Point_List_c* value);
void pyramid_data_model_agra_LineRelativeType_c_free(pyramid_data_model_agra_LineRelativeType_c* value);
void pyramid_data_model_agra_RequirementTargetConstraintsType_c_free(pyramid_data_model_agra_RequirementTargetConstraintsType_c* value);
void pyramid_data_model_agra_AnalyticConstraintsType_c_free(pyramid_data_model_agra_AnalyticConstraintsType_c* value);
void pyramid_data_model_agra_AccessAssessmentFilterType_c_free(pyramid_data_model_agra_AccessAssessmentFilterType_c* value);
void pyramid_data_model_agra_AssetFilterType_c_free(pyramid_data_model_agra_AssetFilterType_c* value);
void pyramid_data_model_agra_SystemFilterType_c_free(pyramid_data_model_agra_SystemFilterType_c* value);
void pyramid_data_model_agra_SystemComparativeType_c_free(pyramid_data_model_agra_SystemComparativeType_c* value);
void pyramid_data_model_agra_SystemCharacteristicType_c_free(pyramid_data_model_agra_SystemCharacteristicType_c* value);
void pyramid_data_model_agra_PrioritizationListValueType_c_free(pyramid_data_model_agra_PrioritizationListValueType_c* value);
void pyramid_data_model_agra_PrioritizationType_c_free(pyramid_data_model_agra_PrioritizationType_c* value);
void pyramid_data_model_agra_RequirementTaxonomyType_c_free(pyramid_data_model_agra_RequirementTaxonomyType_c* value);
void pyramid_data_model_agra_EntityFilterType_c_free(pyramid_data_model_agra_EntityFilterType_c* value);
void pyramid_data_model_agra_DesignationFilterType_c_free(pyramid_data_model_agra_DesignationFilterType_c* value);
void pyramid_data_model_agra_EntityComparativeType_c_free(pyramid_data_model_agra_EntityComparativeType_c* value);
void pyramid_data_model_agra_EntityCharacteristicType_c_free(pyramid_data_model_agra_EntityCharacteristicType_c* value);
void pyramid_data_model_agra_RequirementMetadataType_c_free(pyramid_data_model_agra_RequirementMetadataType_c* value);
void pyramid_data_model_agra_TraceabilityType_c_free(pyramid_data_model_agra_TraceabilityType_c* value);
void pyramid_data_model_agra_CollectionDeckTraceabilityType_c_free(pyramid_data_model_agra_CollectionDeckTraceabilityType_c* value);
void pyramid_data_model_agra_ACTDF_TraceabilityType_c_free(pyramid_data_model_agra_ACTDF_TraceabilityType_c* value);
void pyramid_data_model_agra_ACTDF_CollectionPlanType_c_free(pyramid_data_model_agra_ACTDF_CollectionPlanType_c* value);
void pyramid_data_model_agra_ACTDF_TaskID_Type_c_free(pyramid_data_model_agra_ACTDF_TaskID_Type_c* value);
void pyramid_data_model_agra_EEI_ID_Type_c_free(pyramid_data_model_agra_EEI_ID_Type_c* value);
void pyramid_data_model_agra_ATO_TraceabilityType_c_free(pyramid_data_model_agra_ATO_TraceabilityType_c* value);
void pyramid_data_model_agra_AOCO_TraceabilityType_c_free(pyramid_data_model_agra_AOCO_TraceabilityType_c* value);
void pyramid_data_model_agra_CS_STO_TraceabilityType_c_free(pyramid_data_model_agra_CS_STO_TraceabilityType_c* value);
void pyramid_data_model_agra_RemarksType_c_free(pyramid_data_model_agra_RemarksType_c* value);
void pyramid_data_model_agra_MA_ActionStatusMT_c_free(pyramid_data_model_agra_MA_ActionStatusMT_c* value);
void pyramid_data_model_agra_MA_ActionStatusMDT_c_free(pyramid_data_model_agra_MA_ActionStatusMDT_c* value);
void pyramid_data_model_agra_MA_RequirementExecutionStatusDetailsType_c_free(pyramid_data_model_agra_MA_RequirementExecutionStatusDetailsType_c* value);
void pyramid_data_model_agra_MA_PackageSystemType_c_free(pyramid_data_model_agra_MA_PackageSystemType_c* value);
void pyramid_data_model_agra_CannotComplyType_c_free(pyramid_data_model_agra_CannotComplyType_c* value);
void pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c_free(pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c* value);
void pyramid_data_model_agra_MA_PlansReferenceType_c_free(pyramid_data_model_agra_MA_PlansReferenceType_c* value);
void pyramid_data_model_agra_MA_PlansReferenceBaseType_c_free(pyramid_data_model_agra_MA_PlansReferenceBaseType_c* value);
void pyramid_data_model_agra_TaskPlanID_Type_c_free(pyramid_data_model_agra_TaskPlanID_Type_c* value);
void pyramid_data_model_agra_OrbitPlanID_Type_c_free(pyramid_data_model_agra_OrbitPlanID_Type_c* value);
void pyramid_data_model_agra_OrbitActivityPlanID_Type_c_free(pyramid_data_model_agra_OrbitActivityPlanID_Type_c* value);
void pyramid_data_model_agra_RoutePlanID_Type_c_free(pyramid_data_model_agra_RoutePlanID_Type_c* value);
void pyramid_data_model_agra_RouteActivityPlanID_Type_c_free(pyramid_data_model_agra_RouteActivityPlanID_Type_c* value);
void pyramid_data_model_agra_ActivityPlanID_Type_c_free(pyramid_data_model_agra_ActivityPlanID_Type_c* value);
void pyramid_data_model_agra_EffectPlanID_Type_c_free(pyramid_data_model_agra_EffectPlanID_Type_c* value);
void pyramid_data_model_agra_ActionPlanID_Type_c_free(pyramid_data_model_agra_ActionPlanID_Type_c* value);
void pyramid_data_model_agra_ResponsePlanID_Type_c_free(pyramid_data_model_agra_ResponsePlanID_Type_c* value);
void pyramid_data_model_agra_MA_OpPlanID_Type_c_free(pyramid_data_model_agra_MA_OpPlanID_Type_c* value);
void pyramid_data_model_agra_MissionPlanID_Type_c_free(pyramid_data_model_agra_MissionPlanID_Type_c* value);
void pyramid_data_model_agra_PlannedActivityID_Type_c_free(pyramid_data_model_agra_PlannedActivityID_Type_c* value);
void pyramid_data_model_agra_ActivityID_Type_c_free(pyramid_data_model_agra_ActivityID_Type_c* value);
void pyramid_data_model_agra_MetricsType_c_free(pyramid_data_model_agra_MetricsType_c* value);
void pyramid_data_model_agra_EnduranceType_c_free(pyramid_data_model_agra_EnduranceType_c* value);
void pyramid_data_model_agra_EnduranceBaseType_c_free(pyramid_data_model_agra_EnduranceBaseType_c* value);
void pyramid_data_model_agra_EnduranceFootprintType_c_free(pyramid_data_model_agra_EnduranceFootprintType_c* value);
void pyramid_data_model_agra_BoundaryType_c_free(pyramid_data_model_agra_BoundaryType_c* value);
void pyramid_data_model_agra_ExpendableType_c_free(pyramid_data_model_agra_ExpendableType_c* value);
void pyramid_data_model_agra_WeaponStoreType_c_free(pyramid_data_model_agra_WeaponStoreType_c* value);
void pyramid_data_model_agra_MA_ApprovalPolicyMT_c_free(pyramid_data_model_agra_MA_ApprovalPolicyMT_c* value);
void pyramid_data_model_agra_MA_ApprovalPolicyMDT_c_free(pyramid_data_model_agra_MA_ApprovalPolicyMDT_c* value);
void pyramid_data_model_agra_DataRecordBaseType_c_free(pyramid_data_model_agra_DataRecordBaseType_c* value);
void pyramid_data_model_agra_DataRecordInstanceID_Type_c_free(pyramid_data_model_agra_DataRecordInstanceID_Type_c* value);
void pyramid_data_model_agra_ApprovalPolicyID_Type_c_free(pyramid_data_model_agra_ApprovalPolicyID_Type_c* value);
void pyramid_data_model_agra_MA_PlanPolicyType_c_free(pyramid_data_model_agra_MA_PlanPolicyType_c* value);
void pyramid_data_model_agra_MA_PlanPolicyApplicablePlanType_c_free(pyramid_data_model_agra_MA_PlanPolicyApplicablePlanType_c* value);
void pyramid_data_model_agra_MA_PlanPartsType_c_free(pyramid_data_model_agra_MA_PlanPartsType_c* value);
void pyramid_data_model_agra_MA_PlanPartsBaseType_c_free(pyramid_data_model_agra_MA_PlanPartsBaseType_c* value);
void pyramid_data_model_agra_EffectPlanPartsType_c_free(pyramid_data_model_agra_EffectPlanPartsType_c* value);
void pyramid_data_model_agra_ActionPlanPartsType_c_free(pyramid_data_model_agra_ActionPlanPartsType_c* value);
void pyramid_data_model_agra_MA_TaskPlanPartsType_c_free(pyramid_data_model_agra_MA_TaskPlanPartsType_c* value);
void pyramid_data_model_agra_ResponsePlanPartsType_c_free(pyramid_data_model_agra_ResponsePlanPartsType_c* value);
void pyramid_data_model_agra_MA_OpPlanPartsType_c_free(pyramid_data_model_agra_MA_OpPlanPartsType_c* value);
void pyramid_data_model_agra_OpPointCategoriesType_c_free(pyramid_data_model_agra_OpPointCategoriesType_c* value);
void pyramid_data_model_agra_MA_ActivityPlanPartsType_c_free(pyramid_data_model_agra_MA_ActivityPlanPartsType_c* value);
void pyramid_data_model_agra_RoutePlanPartsType_c_free(pyramid_data_model_agra_RoutePlanPartsType_c* value);
void pyramid_data_model_agra_OrbitPlanPartsType_c_free(pyramid_data_model_agra_OrbitPlanPartsType_c* value);
void pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c_free(pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c* value);
void pyramid_data_model_agra_ConstrainingPlanPartsType_c_free(pyramid_data_model_agra_ConstrainingPlanPartsType_c* value);
void pyramid_data_model_agra_PlanPartsType_c_free(pyramid_data_model_agra_PlanPartsType_c* value);
void pyramid_data_model_agra_PlanPartsBaseType_c_free(pyramid_data_model_agra_PlanPartsBaseType_c* value);
void pyramid_data_model_agra_TaskPlanPartsType_c_free(pyramid_data_model_agra_TaskPlanPartsType_c* value);
void pyramid_data_model_agra_ActivityPlanPartsType_c_free(pyramid_data_model_agra_ActivityPlanPartsType_c* value);
void pyramid_data_model_agra_CommAllocationPartsType_c_free(pyramid_data_model_agra_CommAllocationPartsType_c* value);
void pyramid_data_model_agra_AutonomousPlanningOtherSystemConstrainingPlansType_c_free(pyramid_data_model_agra_AutonomousPlanningOtherSystemConstrainingPlansType_c* value);
void pyramid_data_model_agra_ApprovalPolicyBaseType_c_free(pyramid_data_model_agra_ApprovalPolicyBaseType_c* value);
void pyramid_data_model_agra_DefaultResponseType_c_free(pyramid_data_model_agra_DefaultResponseType_c* value);
void pyramid_data_model_agra_ByResultPolicyType_c_free(pyramid_data_model_agra_ByResultPolicyType_c* value);
void pyramid_data_model_agra_PlanningByResultTriggerType_c_free(pyramid_data_model_agra_PlanningByResultTriggerType_c* value);
void pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c_free(pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c* value);
void pyramid_data_model_agra_PlanVulnerabilityType_c_free(pyramid_data_model_agra_PlanVulnerabilityType_c* value);
void pyramid_data_model_agra_ThreatVulnerabilityByCapabilityType_c_free(pyramid_data_model_agra_ThreatVulnerabilityByCapabilityType_c* value);
void pyramid_data_model_agra_ThresholdVulnerabilityType_c_free(pyramid_data_model_agra_ThresholdVulnerabilityType_c* value);
void pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c_free(pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c* value);
void pyramid_data_model_agra_RequirementTriggerType_c_free(pyramid_data_model_agra_RequirementTriggerType_c* value);
void pyramid_data_model_agra_RankCompareType_c_free(pyramid_data_model_agra_RankCompareType_c* value);
void pyramid_data_model_agra_RequirementTaxonomyDetailedType_c_free(pyramid_data_model_agra_RequirementTaxonomyDetailedType_c* value);
void pyramid_data_model_agra_ByTriggerPolicyType_c_free(pyramid_data_model_agra_ByTriggerPolicyType_c* value);
void pyramid_data_model_agra_PlanningByCaseTriggerType_c_free(pyramid_data_model_agra_PlanningByCaseTriggerType_c* value);
void pyramid_data_model_agra_CommsLostTriggerDataType_c_free(pyramid_data_model_agra_CommsLostTriggerDataType_c* value);
void pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c_free(pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c* value);
void pyramid_data_model_agra_SystemStateFilterType_c_free(pyramid_data_model_agra_SystemStateFilterType_c* value);
void pyramid_data_model_agra_RequirementFailedTriggerType_c_free(pyramid_data_model_agra_RequirementFailedTriggerType_c* value);
void pyramid_data_model_agra_ZoneViolationTriggerDataType_c_free(pyramid_data_model_agra_ZoneViolationTriggerDataType_c* value);
void pyramid_data_model_agra_SatelliteEnduranceType_c_free(pyramid_data_model_agra_SatelliteEnduranceType_c* value);
void pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c_free(pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c* value);
void pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c_free(pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c* value);
void pyramid_data_model_agra_MA_PlanActivationPolicyType_c_free(pyramid_data_model_agra_MA_PlanActivationPolicyType_c* value);
void pyramid_data_model_agra_MissionPlanActivationSettingType_c_free(pyramid_data_model_agra_MissionPlanActivationSettingType_c* value);
void pyramid_data_model_agra_SubPlanActivationSettingType_c_free(pyramid_data_model_agra_SubPlanActivationSettingType_c* value);
void pyramid_data_model_agra_RequirementExecutionPolicyType_c_free(pyramid_data_model_agra_RequirementExecutionPolicyType_c* value);
void pyramid_data_model_agra_ByRequirementPolicyType_c_free(pyramid_data_model_agra_ByRequirementPolicyType_c* value);
void pyramid_data_model_agra_TimedZoneType_c_free(pyramid_data_model_agra_TimedZoneType_c* value);
void pyramid_data_model_agra_ScheduleType_c_free(pyramid_data_model_agra_ScheduleType_c* value);
void pyramid_data_model_agra_ScheduleType_TimeSpan_List_c_free(pyramid_data_model_agra_ScheduleType_TimeSpan_List_c* value);
void pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c_free(pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c* value);
void pyramid_data_model_agra_MissionTraceabilityType_c_free(pyramid_data_model_agra_MissionTraceabilityType_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestMT_c_free(pyramid_data_model_agra_MA_ApprovalRequestMT_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestMDT_c_free(pyramid_data_model_agra_MA_ApprovalRequestMDT_c* value);
void pyramid_data_model_agra_RequestBaseType_c_free(pyramid_data_model_agra_RequestBaseType_c* value);
void pyramid_data_model_agra_RequestID_Type_c_free(pyramid_data_model_agra_RequestID_Type_c* value);
void pyramid_data_model_agra_OperatorRoleType_c_free(pyramid_data_model_agra_OperatorRoleType_c* value);
void pyramid_data_model_agra_OperatorRoleID_Type_c_free(pyramid_data_model_agra_OperatorRoleID_Type_c* value);
void pyramid_data_model_agra_SystemServiceType_c_free(pyramid_data_model_agra_SystemServiceType_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c_free(pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c_free(pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c* value);
void pyramid_data_model_agra_PlanReferenceID_ChoiceType_c_free(pyramid_data_model_agra_PlanReferenceID_ChoiceType_c* value);
void pyramid_data_model_agra_CommScheduleAllocationID_Type_c_free(pyramid_data_model_agra_CommScheduleAllocationID_Type_c* value);
void pyramid_data_model_agra_ApprovalRequestItemType_c_free(pyramid_data_model_agra_ApprovalRequestItemType_c* value);
void pyramid_data_model_agra_DMPI_ID_Type_c_free(pyramid_data_model_agra_DMPI_ID_Type_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c_free(pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c_free(pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c* value);
void pyramid_data_model_agra_MissionPlanActivationType_c_free(pyramid_data_model_agra_MissionPlanActivationType_c* value);
void pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c_free(pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c* value);
void pyramid_data_model_agra_TaskPlanActivationType_c_free(pyramid_data_model_agra_TaskPlanActivationType_c* value);
void pyramid_data_model_agra_RoutePlanActivationType_c_free(pyramid_data_model_agra_RoutePlanActivationType_c* value);
void pyramid_data_model_agra_ActivationPathSegmentType_c_free(pyramid_data_model_agra_ActivationPathSegmentType_c* value);
void pyramid_data_model_agra_PathID_Type_c_free(pyramid_data_model_agra_PathID_Type_c* value);
void pyramid_data_model_agra_SegmentID_Type_c_free(pyramid_data_model_agra_SegmentID_Type_c* value);
void pyramid_data_model_agra_RouteActivityPlanActivationType_c_free(pyramid_data_model_agra_RouteActivityPlanActivationType_c* value);
void pyramid_data_model_agra_OrbitPlanActivationType_c_free(pyramid_data_model_agra_OrbitPlanActivationType_c* value);
void pyramid_data_model_agra_ActivationOrbitSequenceType_c_free(pyramid_data_model_agra_ActivationOrbitSequenceType_c* value);
void pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c_free(pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c* value);
void pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c_free(pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c* value);
void pyramid_data_model_agra_OrbitActivityPlanActivationType_c_free(pyramid_data_model_agra_OrbitActivityPlanActivationType_c* value);
void pyramid_data_model_agra_ActivityPlanActivationType_c_free(pyramid_data_model_agra_ActivityPlanActivationType_c* value);
void pyramid_data_model_agra_EffectPlanActivationType_c_free(pyramid_data_model_agra_EffectPlanActivationType_c* value);
void pyramid_data_model_agra_ActionPlanActivationType_c_free(pyramid_data_model_agra_ActionPlanActivationType_c* value);
void pyramid_data_model_agra_ResponsePlanActivationType_c_free(pyramid_data_model_agra_ResponsePlanActivationType_c* value);
void pyramid_data_model_agra_MA_OpPlanActivationType_c_free(pyramid_data_model_agra_MA_OpPlanActivationType_c* value);
void pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c_free(pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c* value);
void pyramid_data_model_agra_ExecutionPlanSetID_Type_c_free(pyramid_data_model_agra_ExecutionPlanSetID_Type_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c_free(pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c_free(pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c* value);
void pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c_free(pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c* value);
void pyramid_data_model_agra_RequestStatusBaseType_c_free(pyramid_data_model_agra_RequestStatusBaseType_c* value);
void pyramid_data_model_agra_OperatorID_Type_c_free(pyramid_data_model_agra_OperatorID_Type_c* value);
void pyramid_data_model_agra_MA_MissionPlanMT_c_free(pyramid_data_model_agra_MA_MissionPlanMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanMDT_c_free(pyramid_data_model_agra_MA_MissionPlanMDT_c* value);
void pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c_free(pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c* value);
void pyramid_data_model_agra_MissionPlanCommandID_Type_c_free(pyramid_data_model_agra_MissionPlanCommandID_Type_c* value);
void pyramid_data_model_agra_MA_PlanApplicabilityType_c_free(pyramid_data_model_agra_MA_PlanApplicabilityType_c* value);
void pyramid_data_model_agra_ExecutionSequenceType_c_free(pyramid_data_model_agra_ExecutionSequenceType_c* value);
void pyramid_data_model_agra_ExecutionSequencePlanSetsType_c_free(pyramid_data_model_agra_ExecutionSequencePlanSetsType_c* value);
void pyramid_data_model_agra_RouteExecutionPlanSetType_c_free(pyramid_data_model_agra_RouteExecutionPlanSetType_c* value);
void pyramid_data_model_agra_ExecutionPlanSetBaseType_c_free(pyramid_data_model_agra_ExecutionPlanSetBaseType_c* value);
void pyramid_data_model_agra_OrbitExecutionPlanSetType_c_free(pyramid_data_model_agra_OrbitExecutionPlanSetType_c* value);
void pyramid_data_model_agra_MA_MissionPlanInputsType_c_free(pyramid_data_model_agra_MA_MissionPlanInputsType_c* value);
void pyramid_data_model_agra_MA_PlanInputsCoreType_c_free(pyramid_data_model_agra_MA_PlanInputsCoreType_c* value);
void pyramid_data_model_agra_PlanningProcessID_Type_c_free(pyramid_data_model_agra_PlanningProcessID_Type_c* value);
void pyramid_data_model_agra_MA_ReplanReasonType_c_free(pyramid_data_model_agra_MA_ReplanReasonType_c* value);
void pyramid_data_model_agra_MA_PlanningTriggerType_c_free(pyramid_data_model_agra_MA_PlanningTriggerType_c* value);
void pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c_free(pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c* value);
void pyramid_data_model_agra_MA_RequirementTriggerType_c_free(pyramid_data_model_agra_MA_RequirementTriggerType_c* value);
void pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c_free(pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c* value);
void pyramid_data_model_agra_MA_CapabilityTaxonomyType_c_free(pyramid_data_model_agra_MA_CapabilityTaxonomyType_c* value);
void pyramid_data_model_agra_MA_RequirementFailedTriggerType_c_free(pyramid_data_model_agra_MA_RequirementFailedTriggerType_c* value);
void pyramid_data_model_agra_MA_PlanningByResultTriggerType_c_free(pyramid_data_model_agra_MA_PlanningByResultTriggerType_c* value);
void pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c_free(pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c* value);
void pyramid_data_model_agra_AutonomousPlanningActionID_Type_c_free(pyramid_data_model_agra_AutonomousPlanningActionID_Type_c* value);
void pyramid_data_model_agra_MissionContingencyAlertID_Type_c_free(pyramid_data_model_agra_MissionContingencyAlertID_Type_c* value);
void pyramid_data_model_agra_MA_MissionEnvironmentConstraintType_c_free(pyramid_data_model_agra_MA_MissionEnvironmentConstraintType_c* value);
void pyramid_data_model_agra_ConstrainedEntityType_c_free(pyramid_data_model_agra_ConstrainedEntityType_c* value);
void pyramid_data_model_agra_ConstrainedOpPointType_c_free(pyramid_data_model_agra_ConstrainedOpPointType_c* value);
void pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c_free(pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c* value);
void pyramid_data_model_agra_EmergencyReferencePointType_c_free(pyramid_data_model_agra_EmergencyReferencePointType_c* value);
void pyramid_data_model_agra_OpPointReferenceType_c_free(pyramid_data_model_agra_OpPointReferenceType_c* value);
void pyramid_data_model_agra_GeoLocatedStoredObjectType_c_free(pyramid_data_model_agra_GeoLocatedStoredObjectType_c* value);
void pyramid_data_model_agra_EntityDataType_c_free(pyramid_data_model_agra_EntityDataType_c* value);
void pyramid_data_model_agra_EntityMDT_c_free(pyramid_data_model_agra_EntityMDT_c* value);
void pyramid_data_model_agra_DateTimeSigmaType_c_free(pyramid_data_model_agra_DateTimeSigmaType_c* value);
void pyramid_data_model_agra_EntitySourceType_c_free(pyramid_data_model_agra_EntitySourceType_c* value);
void pyramid_data_model_agra_EntitySourceSpecificDataType_c_free(pyramid_data_model_agra_EntitySourceSpecificDataType_c* value);
void pyramid_data_model_agra_CorrelatedEntityID_Type_c_free(pyramid_data_model_agra_CorrelatedEntityID_Type_c* value);
void pyramid_data_model_agra_DropRestrictionType_c_free(pyramid_data_model_agra_DropRestrictionType_c* value);
void pyramid_data_model_agra_AssociatedDropRestrictMessageType_c_free(pyramid_data_model_agra_AssociatedDropRestrictMessageType_c* value);
void pyramid_data_model_agra_EntitySourceIdentifierType_c_free(pyramid_data_model_agra_EntitySourceIdentifierType_c* value);
void pyramid_data_model_agra_EOB_RecordID_Type_c_free(pyramid_data_model_agra_EOB_RecordID_Type_c* value);
void pyramid_data_model_agra_EntityExternalType_c_free(pyramid_data_model_agra_EntityExternalType_c* value);
void pyramid_data_model_agra_EntityFusionSourceType_c_free(pyramid_data_model_agra_EntityFusionSourceType_c* value);
void pyramid_data_model_agra_EntityContributorID_ChoiceType_c_free(pyramid_data_model_agra_EntityContributorID_ChoiceType_c* value);
void pyramid_data_model_agra_EOB_EmitterID_Type_c_free(pyramid_data_model_agra_EOB_EmitterID_Type_c* value);
void pyramid_data_model_agra_SOB_C2_RecordID_Type_c_free(pyramid_data_model_agra_SOB_C2_RecordID_Type_c* value);
void pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c_free(pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c* value);
void pyramid_data_model_agra_MeasurementID_Type_c_free(pyramid_data_model_agra_MeasurementID_Type_c* value);
void pyramid_data_model_agra_IdentityConfidenceType_c_free(pyramid_data_model_agra_IdentityConfidenceType_c* value);
void pyramid_data_model_agra_StandardIdentityConfidenceType_c_free(pyramid_data_model_agra_StandardIdentityConfidenceType_c* value);
void pyramid_data_model_agra_ExerciseIdentityType_c_free(pyramid_data_model_agra_ExerciseIdentityType_c* value);
void pyramid_data_model_agra_EnvironmentIdentityConfidenceType_c_free(pyramid_data_model_agra_EnvironmentIdentityConfidenceType_c* value);
void pyramid_data_model_agra_PlatformIdentityConfidenceType_c_free(pyramid_data_model_agra_PlatformIdentityConfidenceType_c* value);
void pyramid_data_model_agra_SpecificIdentityConfidenceType_c_free(pyramid_data_model_agra_SpecificIdentityConfidenceType_c* value);
void pyramid_data_model_agra_EmitterMultipleType_c_free(pyramid_data_model_agra_EmitterMultipleType_c* value);
void pyramid_data_model_agra_EmitterIdentityConfidenceType_c_free(pyramid_data_model_agra_EmitterIdentityConfidenceType_c* value);
void pyramid_data_model_agra_SpecificEmitterMultipleType_c_free(pyramid_data_model_agra_SpecificEmitterMultipleType_c* value);
void pyramid_data_model_agra_SpecificEmitterIdentityConfidenceType_c_free(pyramid_data_model_agra_SpecificEmitterIdentityConfidenceType_c* value);
void pyramid_data_model_agra_SpecificVehicleIdentityConfidenceType_c_free(pyramid_data_model_agra_SpecificVehicleIdentityConfidenceType_c* value);
void pyramid_data_model_agra_SpecificFacilityIdentityConfidenceType_c_free(pyramid_data_model_agra_SpecificFacilityIdentityConfidenceType_c* value);
void pyramid_data_model_agra_EOB_IdentityConfidenceType_c_free(pyramid_data_model_agra_EOB_IdentityConfidenceType_c* value);
void pyramid_data_model_agra_StoreMultipleType_c_free(pyramid_data_model_agra_StoreMultipleType_c* value);
void pyramid_data_model_agra_StoreConfidenceType_c_free(pyramid_data_model_agra_StoreConfidenceType_c* value);
void pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c_free(pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c* value);
void pyramid_data_model_agra_EntityCapabilitySourceType_c_free(pyramid_data_model_agra_EntityCapabilitySourceType_c* value);
void pyramid_data_model_agra_ProductMetadataID_Type_c_free(pyramid_data_model_agra_ProductMetadataID_Type_c* value);
void pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c_free(pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c* value);
void pyramid_data_model_agra_EntityIdentityType_c_free(pyramid_data_model_agra_EntityIdentityType_c* value);
void pyramid_data_model_agra_QualifyingTagsType_c_free(pyramid_data_model_agra_QualifyingTagsType_c* value);
void pyramid_data_model_agra_SystemMessageIdentifierType_c_free(pyramid_data_model_agra_SystemMessageIdentifierType_c* value);
void pyramid_data_model_agra_NetworkLinkID_Type_c_free(pyramid_data_model_agra_NetworkLinkID_Type_c* value);
void pyramid_data_model_agra_TimeFunctionType_c_free(pyramid_data_model_agra_TimeFunctionType_c* value);
void pyramid_data_model_agra_PlatformStatusType_c_free(pyramid_data_model_agra_PlatformStatusType_c* value);
void pyramid_data_model_agra_PlatformFunctionStatusType_c_free(pyramid_data_model_agra_PlatformFunctionStatusType_c* value);
void pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c_free(pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c* value);
void pyramid_data_model_agra_PlatformStatusSAM_Type_c_free(pyramid_data_model_agra_PlatformStatusSAM_Type_c* value);
void pyramid_data_model_agra_SurfaceRecoveryType_c_free(pyramid_data_model_agra_SurfaceRecoveryType_c* value);
void pyramid_data_model_agra_DatalinkControlType_c_free(pyramid_data_model_agra_DatalinkControlType_c* value);
void pyramid_data_model_agra_EntitySignalSummaryType_c_free(pyramid_data_model_agra_EntitySignalSummaryType_c* value);
void pyramid_data_model_agra_SignalSummaryType_c_free(pyramid_data_model_agra_SignalSummaryType_c* value);
void pyramid_data_model_agra_FrequencyRangeType_c_free(pyramid_data_model_agra_FrequencyRangeType_c* value);
void pyramid_data_model_agra_FrequencyControlType_c_free(pyramid_data_model_agra_FrequencyControlType_c* value);
void pyramid_data_model_agra_PulseDataID_Type_c_free(pyramid_data_model_agra_PulseDataID_Type_c* value);
void pyramid_data_model_agra_StrengthType_c_free(pyramid_data_model_agra_StrengthType_c* value);
void pyramid_data_model_agra_StrengthRangeType_c_free(pyramid_data_model_agra_StrengthRangeType_c* value);
void pyramid_data_model_agra_ActivityAgainstType_c_free(pyramid_data_model_agra_ActivityAgainstType_c* value);
void pyramid_data_model_agra_ActivityActorID_ChoiceType_c_free(pyramid_data_model_agra_ActivityActorID_ChoiceType_c* value);
void pyramid_data_model_agra_VoiceControlType_c_free(pyramid_data_model_agra_VoiceControlType_c* value);
void pyramid_data_model_agra_EntityRemoveInfoType_c_free(pyramid_data_model_agra_EntityRemoveInfoType_c* value);
void pyramid_data_model_agra_OrbitalSingleVectorParametersType_c_free(pyramid_data_model_agra_OrbitalSingleVectorParametersType_c* value);
void pyramid_data_model_agra_EphemerisOrbitalModelType_c_free(pyramid_data_model_agra_EphemerisOrbitalModelType_c* value);
void pyramid_data_model_agra_OrbitalModelType_c_free(pyramid_data_model_agra_OrbitalModelType_c* value);
void pyramid_data_model_agra_EntityMetadataMDT_c_free(pyramid_data_model_agra_EntityMetadataMDT_c* value);
void pyramid_data_model_agra_MetadataID_Type_c_free(pyramid_data_model_agra_MetadataID_Type_c* value);
void pyramid_data_model_agra_EntityMetadataPET_c_free(pyramid_data_model_agra_EntityMetadataPET_c* value);
void pyramid_data_model_agra_SystemDataType_c_free(pyramid_data_model_agra_SystemDataType_c* value);
void pyramid_data_model_agra_SystemStatusMDT_c_free(pyramid_data_model_agra_SystemStatusMDT_c* value);
void pyramid_data_model_agra_SystemIdentityType_c_free(pyramid_data_model_agra_SystemIdentityType_c* value);
void pyramid_data_model_agra_SystemCommunicationsType_c_free(pyramid_data_model_agra_SystemCommunicationsType_c* value);
void pyramid_data_model_agra_CommSystemUsageType_c_free(pyramid_data_model_agra_CommSystemUsageType_c* value);
void pyramid_data_model_agra_CommSystemID_Type_c_free(pyramid_data_model_agra_CommSystemID_Type_c* value);
void pyramid_data_model_agra_CommModeUsageType_c_free(pyramid_data_model_agra_CommModeUsageType_c* value);
void pyramid_data_model_agra_CommAntennaModeType_c_free(pyramid_data_model_agra_CommAntennaModeType_c* value);
void pyramid_data_model_agra_SubsystemID_Type_c_free(pyramid_data_model_agra_SubsystemID_Type_c* value);
void pyramid_data_model_agra_PositionReportMDT_c_free(pyramid_data_model_agra_PositionReportMDT_c* value);
void pyramid_data_model_agra_InertialStateType_c_free(pyramid_data_model_agra_InertialStateType_c* value);
void pyramid_data_model_agra_Point4D_Type_c_free(pyramid_data_model_agra_Point4D_Type_c* value);
void pyramid_data_model_agra_Velocity3D_Type_c_free(pyramid_data_model_agra_Velocity3D_Type_c* value);
void pyramid_data_model_agra_OrientationRateType_c_free(pyramid_data_model_agra_OrientationRateType_c* value);
void pyramid_data_model_agra_NavigationReportMDT_c_free(pyramid_data_model_agra_NavigationReportMDT_c* value);
void pyramid_data_model_agra_NavigationType_c_free(pyramid_data_model_agra_NavigationType_c* value);
void pyramid_data_model_agra_NavigationSourceType_c_free(pyramid_data_model_agra_NavigationSourceType_c* value);
void pyramid_data_model_agra_MissionPlanNavigationType_c_free(pyramid_data_model_agra_MissionPlanNavigationType_c* value);
void pyramid_data_model_agra_Point3D_Type_c_free(pyramid_data_model_agra_Point3D_Type_c* value);
void pyramid_data_model_agra_RelativeNavigationType_c_free(pyramid_data_model_agra_RelativeNavigationType_c* value);
void pyramid_data_model_agra_SlavedNavigationType_c_free(pyramid_data_model_agra_SlavedNavigationType_c* value);
void pyramid_data_model_agra_SystemMetadataMDT_c_free(pyramid_data_model_agra_SystemMetadataMDT_c* value);
void pyramid_data_model_agra_SystemMetadataPET_c_free(pyramid_data_model_agra_SystemMetadataPET_c* value);
void pyramid_data_model_agra_EmergencyReferenceOpPointType_c_free(pyramid_data_model_agra_EmergencyReferenceOpPointType_c* value);
void pyramid_data_model_agra_OpPointBaseType_c_free(pyramid_data_model_agra_OpPointBaseType_c* value);
void pyramid_data_model_agra_OpBaseType_c_free(pyramid_data_model_agra_OpBaseType_c* value);
void pyramid_data_model_agra_OpDescriptionType_c_free(pyramid_data_model_agra_OpDescriptionType_c* value);
void pyramid_data_model_agra_SystemScheduleStateType_c_free(pyramid_data_model_agra_SystemScheduleStateType_c* value);
void pyramid_data_model_agra_ScheduleStateType_c_free(pyramid_data_model_agra_ScheduleStateType_c* value);
void pyramid_data_model_agra_OpPointChoiceType_c_free(pyramid_data_model_agra_OpPointChoiceType_c* value);
void pyramid_data_model_agra_OpPointPositionType_c_free(pyramid_data_model_agra_OpPointPositionType_c* value);
void pyramid_data_model_agra_KinematicsFixedPositionType_c_free(pyramid_data_model_agra_KinematicsFixedPositionType_c* value);
void pyramid_data_model_agra_SystemConfigurationType_c_free(pyramid_data_model_agra_SystemConfigurationType_c* value);
void pyramid_data_model_agra_VehicleConfigurationType_c_free(pyramid_data_model_agra_VehicleConfigurationType_c* value);
void pyramid_data_model_agra_InertialStateRelativeType_c_free(pyramid_data_model_agra_InertialStateRelativeType_c* value);
void pyramid_data_model_agra_PointChoice4D_Type_c_free(pyramid_data_model_agra_PointChoice4D_Type_c* value);
void pyramid_data_model_agra_Point4D_RelativeType_c_free(pyramid_data_model_agra_Point4D_RelativeType_c* value);
void pyramid_data_model_agra_RelativeOffset3D_Type_c_free(pyramid_data_model_agra_RelativeOffset3D_Type_c* value);
void pyramid_data_model_agra_SafeAltitudeType_c_free(pyramid_data_model_agra_SafeAltitudeType_c* value);
void pyramid_data_model_agra_AltitudeRadialPairType_c_free(pyramid_data_model_agra_AltitudeRadialPairType_c* value);
void pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c_free(pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c* value);
void pyramid_data_model_agra_OpLineMDT_c_free(pyramid_data_model_agra_OpLineMDT_c* value);
void pyramid_data_model_agra_OpLineType_c_free(pyramid_data_model_agra_OpLineType_c* value);
void pyramid_data_model_agra_OpZoneMDT_c_free(pyramid_data_model_agra_OpZoneMDT_c* value);
void pyramid_data_model_agra_OpZoneCategoryType_c_free(pyramid_data_model_agra_OpZoneCategoryType_c* value);
void pyramid_data_model_agra_ConstrainedEntryExitType_c_free(pyramid_data_model_agra_ConstrainedEntryExitType_c* value);
void pyramid_data_model_agra_OpZoneFilterAreaPET_c_free(pyramid_data_model_agra_OpZoneFilterAreaPET_c* value);
void pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c_free(pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c* value);
void pyramid_data_model_agra_OpZoneJammingType_c_free(pyramid_data_model_agra_OpZoneJammingType_c* value);
void pyramid_data_model_agra_IngressEgressType_c_free(pyramid_data_model_agra_IngressEgressType_c* value);
void pyramid_data_model_agra_OpZoneMissileDataType_c_free(pyramid_data_model_agra_OpZoneMissileDataType_c* value);
void pyramid_data_model_agra_TrackNumberOrEntityType_c_free(pyramid_data_model_agra_TrackNumberOrEntityType_c* value);
void pyramid_data_model_agra_Link16TrackIdentifierType_c_free(pyramid_data_model_agra_Link16TrackIdentifierType_c* value);
void pyramid_data_model_agra_OpZoneNoFireType_c_free(pyramid_data_model_agra_OpZoneNoFireType_c* value);
void pyramid_data_model_agra_OpZoneNoFlyType_c_free(pyramid_data_model_agra_OpZoneNoFlyType_c* value);
void pyramid_data_model_agra_VehicleCommandDataType_c_free(pyramid_data_model_agra_VehicleCommandDataType_c* value);
void pyramid_data_model_agra_CommAllocationActionType_c_free(pyramid_data_model_agra_CommAllocationActionType_c* value);
void pyramid_data_model_agra_OpZoneWeaponRestrictionType_c_free(pyramid_data_model_agra_OpZoneWeaponRestrictionType_c* value);
void pyramid_data_model_agra_WeaponRestrictionType_c_free(pyramid_data_model_agra_WeaponRestrictionType_c* value);
void pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c_free(pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c* value);
void pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c_free(pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c* value);
void pyramid_data_model_agra_OpZoneWeatherType_c_free(pyramid_data_model_agra_OpZoneWeatherType_c* value);
void pyramid_data_model_agra_WeatherAreaDataType_c_free(pyramid_data_model_agra_WeatherAreaDataType_c* value);
void pyramid_data_model_agra_CloudsType_c_free(pyramid_data_model_agra_CloudsType_c* value);
void pyramid_data_model_agra_WeatherEffectsType_c_free(pyramid_data_model_agra_WeatherEffectsType_c* value);
void pyramid_data_model_agra_WindDataType_c_free(pyramid_data_model_agra_WindDataType_c* value);
void pyramid_data_model_agra_WindDataChoiceType_c_free(pyramid_data_model_agra_WindDataChoiceType_c* value);
void pyramid_data_model_agra_WindMagnitudeType_c_free(pyramid_data_model_agra_WindMagnitudeType_c* value);
void pyramid_data_model_agra_OpZoneType_c_free(pyramid_data_model_agra_OpZoneType_c* value);
void pyramid_data_model_agra_OpVolumeMDT_c_free(pyramid_data_model_agra_OpVolumeMDT_c* value);
void pyramid_data_model_agra_GeoLocatedObjectType_c_free(pyramid_data_model_agra_GeoLocatedObjectType_c* value);
void pyramid_data_model_agra_SignalReportID_Type_c_free(pyramid_data_model_agra_SignalReportID_Type_c* value);
void pyramid_data_model_agra_Link16HazardType_c_free(pyramid_data_model_agra_Link16HazardType_c* value);
void pyramid_data_model_agra_ConstrainedOpLineType_c_free(pyramid_data_model_agra_ConstrainedOpLineType_c* value);
void pyramid_data_model_agra_ConstrainedOpZoneType_c_free(pyramid_data_model_agra_ConstrainedOpZoneType_c* value);
void pyramid_data_model_agra_ConstrainedOpVolumeType_c_free(pyramid_data_model_agra_ConstrainedOpVolumeType_c* value);
void pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c_free(pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c* value);
void pyramid_data_model_agra_MA_RequirementChoiceType_c_free(pyramid_data_model_agra_MA_RequirementChoiceType_c* value);
void pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c_free(pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c* value);
void pyramid_data_model_agra_ParameterAssertType_c_free(pyramid_data_model_agra_ParameterAssertType_c* value);
void pyramid_data_model_agra_ParameterID_Type_c_free(pyramid_data_model_agra_ParameterID_Type_c* value);
void pyramid_data_model_agra_ParameterValueType_c_free(pyramid_data_model_agra_ParameterValueType_c* value);
void pyramid_data_model_agra_MA_RequirementPlanningCandidateType_c_free(pyramid_data_model_agra_MA_RequirementPlanningCandidateType_c* value);
void pyramid_data_model_agra_MA_PlanningCandidateBaseType_c_free(pyramid_data_model_agra_MA_PlanningCandidateBaseType_c* value);
void pyramid_data_model_agra_MA_ConstrainingPlansType_c_free(pyramid_data_model_agra_MA_ConstrainingPlansType_c* value);
void pyramid_data_model_agra_MA_TaskPlanConstraintType_c_free(pyramid_data_model_agra_MA_TaskPlanConstraintType_c* value);
void pyramid_data_model_agra_RoutePlanConstraintType_c_free(pyramid_data_model_agra_RoutePlanConstraintType_c* value);
void pyramid_data_model_agra_RouteActivityPlanConstraintType_c_free(pyramid_data_model_agra_RouteActivityPlanConstraintType_c* value);
void pyramid_data_model_agra_OrbitPlanConstraintType_c_free(pyramid_data_model_agra_OrbitPlanConstraintType_c* value);
void pyramid_data_model_agra_OrbitActivityPlanConstraintType_c_free(pyramid_data_model_agra_OrbitActivityPlanConstraintType_c* value);
void pyramid_data_model_agra_ActivityPlanConstraintType_c_free(pyramid_data_model_agra_ActivityPlanConstraintType_c* value);
void pyramid_data_model_agra_CommAllocationConstraintType_c_free(pyramid_data_model_agra_CommAllocationConstraintType_c* value);
void pyramid_data_model_agra_EffectPlanConstraintType_c_free(pyramid_data_model_agra_EffectPlanConstraintType_c* value);
void pyramid_data_model_agra_ActionPlanConstraintType_c_free(pyramid_data_model_agra_ActionPlanConstraintType_c* value);
void pyramid_data_model_agra_ResponsePlanConstraintType_c_free(pyramid_data_model_agra_ResponsePlanConstraintType_c* value);
void pyramid_data_model_agra_MA_OtherSystemConstrainingPlansType_c_free(pyramid_data_model_agra_MA_OtherSystemConstrainingPlansType_c* value);
void pyramid_data_model_agra_PlanningGuidelineType_c_free(pyramid_data_model_agra_PlanningGuidelineType_c* value);
void pyramid_data_model_agra_PlanningPointType_c_free(pyramid_data_model_agra_PlanningPointType_c* value);
void pyramid_data_model_agra_PlanningLocationType_c_free(pyramid_data_model_agra_PlanningLocationType_c* value);
void pyramid_data_model_agra_RoutePlanReferenceType_c_free(pyramid_data_model_agra_RoutePlanReferenceType_c* value);
void pyramid_data_model_agra_PlanningPointPriorityType_c_free(pyramid_data_model_agra_PlanningPointPriorityType_c* value);
void pyramid_data_model_agra_OrbitGuidelineType_c_free(pyramid_data_model_agra_OrbitGuidelineType_c* value);
void pyramid_data_model_agra_OrbitPlanningStateType_c_free(pyramid_data_model_agra_OrbitPlanningStateType_c* value);
void pyramid_data_model_agra_OrbitTransitionSequenceType_c_free(pyramid_data_model_agra_OrbitTransitionSequenceType_c* value);
void pyramid_data_model_agra_MA_RequirementPlanConstraintType_c_free(pyramid_data_model_agra_MA_RequirementPlanConstraintType_c* value);
void pyramid_data_model_agra_MA_RequirementAllocationCommandType_c_free(pyramid_data_model_agra_MA_RequirementAllocationCommandType_c* value);
void pyramid_data_model_agra_MA_EffectAllocationType_c_free(pyramid_data_model_agra_MA_EffectAllocationType_c* value);
void pyramid_data_model_agra_MA_RequirementAllocationBaseType_c_free(pyramid_data_model_agra_MA_RequirementAllocationBaseType_c* value);
void pyramid_data_model_agra_RequirementAllocationDetailsType_c_free(pyramid_data_model_agra_RequirementAllocationDetailsType_c* value);
void pyramid_data_model_agra_DMPI_AllocationType_c_free(pyramid_data_model_agra_DMPI_AllocationType_c* value);
void pyramid_data_model_agra_AO_CodeType_c_free(pyramid_data_model_agra_AO_CodeType_c* value);
void pyramid_data_model_agra_MA_ActionAllocationType_c_free(pyramid_data_model_agra_MA_ActionAllocationType_c* value);
void pyramid_data_model_agra_MA_TaskAllocationType_c_free(pyramid_data_model_agra_MA_TaskAllocationType_c* value);
void pyramid_data_model_agra_MA_ResponseAllocationType_c_free(pyramid_data_model_agra_MA_ResponseAllocationType_c* value);
void pyramid_data_model_agra_RequirementAssociationConstraintType_c_free(pyramid_data_model_agra_RequirementAssociationConstraintType_c* value);
void pyramid_data_model_agra_AssociatedRequirementsType_c_free(pyramid_data_model_agra_AssociatedRequirementsType_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c_free(pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c_free(pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c* value);
void pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c_free(pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c_free(pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c_free(pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c* value);
void pyramid_data_model_agra_PlanCommandStatusType_c_free(pyramid_data_model_agra_PlanCommandStatusType_c* value);
void pyramid_data_model_agra_CompletionStatusType_c_free(pyramid_data_model_agra_CompletionStatusType_c* value);
void pyramid_data_model_agra_IncompleteProcessingType_c_free(pyramid_data_model_agra_IncompleteProcessingType_c* value);
void pyramid_data_model_agra_MA_PlanActivationStatusType_c_free(pyramid_data_model_agra_MA_PlanActivationStatusType_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c_free(pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c_free(pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c* value);
void pyramid_data_model_agra_MA_PlanActivationStateType_c_free(pyramid_data_model_agra_MA_PlanActivationStateType_c* value);
void pyramid_data_model_agra_MA_MissionPlanCommandMT_c_free(pyramid_data_model_agra_MA_MissionPlanCommandMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanCommandMDT_c_free(pyramid_data_model_agra_MA_MissionPlanCommandMDT_c* value);
void pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c_free(pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c_free(pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c* value);
void pyramid_data_model_agra_MA_RequirementPlanningResultType_c_free(pyramid_data_model_agra_MA_RequirementPlanningResultType_c* value);
void pyramid_data_model_agra_MA_RequirementPlanningResultBaseType_c_free(pyramid_data_model_agra_MA_RequirementPlanningResultBaseType_c* value);
void pyramid_data_model_agra_MA_PlanReferenceType_c_free(pyramid_data_model_agra_MA_PlanReferenceType_c* value);
void pyramid_data_model_agra_UnallocatedReasonType_c_free(pyramid_data_model_agra_UnallocatedReasonType_c* value);
void pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c_free(pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c* value);
void pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c_free(pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c* value);
void pyramid_data_model_agra_MA_MissionPlanExecutionStateType_c_free(pyramid_data_model_agra_MA_MissionPlanExecutionStateType_c* value);
void pyramid_data_model_agra_MA_ExecutionPlanSetExecutionStateType_c_free(pyramid_data_model_agra_MA_ExecutionPlanSetExecutionStateType_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionMT_c_free(pyramid_data_model_agra_MA_PlanningFunctionMT_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionMDT_c_free(pyramid_data_model_agra_MA_PlanningFunctionMDT_c* value);
void pyramid_data_model_agra_PlanningFunctionID_Type_c_free(pyramid_data_model_agra_PlanningFunctionID_Type_c* value);
void pyramid_data_model_agra_PlanningInterfacesType_c_free(pyramid_data_model_agra_PlanningInterfacesType_c* value);
void pyramid_data_model_agra_PlanningInterfaceType_c_free(pyramid_data_model_agra_PlanningInterfaceType_c* value);
void pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c_free(pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c* value);
void pyramid_data_model_agra_MA_MissionPlanProcessType_c_free(pyramid_data_model_agra_MA_MissionPlanProcessType_c* value);
void pyramid_data_model_agra_MA_MissionPlanProcessDescriptionType_c_free(pyramid_data_model_agra_MA_MissionPlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_PlanningDiscoveryBaseType_c_free(pyramid_data_model_agra_PlanningDiscoveryBaseType_c* value);
void pyramid_data_model_agra_PlanningApplicabilitySystemType_c_free(pyramid_data_model_agra_PlanningApplicabilitySystemType_c* value);
void pyramid_data_model_agra_ApplicabilityType_c_free(pyramid_data_model_agra_ApplicabilityType_c* value);
void pyramid_data_model_agra_TaskPlanProcessType_c_free(pyramid_data_model_agra_TaskPlanProcessType_c* value);
void pyramid_data_model_agra_TaskPlanProcessDescriptionType_c_free(pyramid_data_model_agra_TaskPlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_RoutePlanProcessType_c_free(pyramid_data_model_agra_RoutePlanProcessType_c* value);
void pyramid_data_model_agra_RoutePlanProcessDescriptionType_c_free(pyramid_data_model_agra_RoutePlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_ActivityPlanProcessType_c_free(pyramid_data_model_agra_ActivityPlanProcessType_c* value);
void pyramid_data_model_agra_ActivityPlanProcessDescriptionType_c_free(pyramid_data_model_agra_ActivityPlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_OrbitPlanProcessType_c_free(pyramid_data_model_agra_OrbitPlanProcessType_c* value);
void pyramid_data_model_agra_OrbitPlanProcessDescriptionType_c_free(pyramid_data_model_agra_OrbitPlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_EffectPlanProcessType_c_free(pyramid_data_model_agra_EffectPlanProcessType_c* value);
void pyramid_data_model_agra_EffectPlanProcessDescriptionType_c_free(pyramid_data_model_agra_EffectPlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_ActionPlanProcessType_c_free(pyramid_data_model_agra_ActionPlanProcessType_c* value);
void pyramid_data_model_agra_ActionPlanProcessDescriptionType_c_free(pyramid_data_model_agra_ActionPlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_ResponsePlanProcessType_c_free(pyramid_data_model_agra_ResponsePlanProcessType_c* value);
void pyramid_data_model_agra_ResponsePlanProcessDescriptionType_c_free(pyramid_data_model_agra_ResponsePlanProcessDescriptionType_c* value);
void pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c_free(pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c* value);
void pyramid_data_model_agra_PlanScoringProcessType_c_free(pyramid_data_model_agra_PlanScoringProcessType_c* value);
void pyramid_data_model_agra_ScoringProcessID_Type_c_free(pyramid_data_model_agra_ScoringProcessID_Type_c* value);
void pyramid_data_model_agra_PlanScoringDescriptionType_c_free(pyramid_data_model_agra_PlanScoringDescriptionType_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c_free(pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c_free(pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c* value);
void pyramid_data_model_agra_CommandBaseType_c_free(pyramid_data_model_agra_CommandBaseType_c* value);
void pyramid_data_model_agra_PlanInterfaceCommandType_c_free(pyramid_data_model_agra_PlanInterfaceCommandType_c* value);
void pyramid_data_model_agra_PlanInterfaceStateType_c_free(pyramid_data_model_agra_PlanInterfaceStateType_c* value);
void pyramid_data_model_agra_MA_MissionPlanningAutonomySettingType_c_free(pyramid_data_model_agra_MA_MissionPlanningAutonomySettingType_c* value);
void pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c_free(pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c* value);
void pyramid_data_model_agra_RequirementFilterType_c_free(pyramid_data_model_agra_RequirementFilterType_c* value);
void pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c_free(pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c* value);
void pyramid_data_model_agra_OperatorLocationOfInterestComparativeType_c_free(pyramid_data_model_agra_OperatorLocationOfInterestComparativeType_c* value);
void pyramid_data_model_agra_QueryMessageType_c_free(pyramid_data_model_agra_QueryMessageType_c* value);
void pyramid_data_model_agra_MA_InertialStateType_c_free(pyramid_data_model_agra_MA_InertialStateType_c* value);
void pyramid_data_model_agra_WeatherReportGridDataType_c_free(pyramid_data_model_agra_WeatherReportGridDataType_c* value);
void pyramid_data_model_agra_WayPointType_c_free(pyramid_data_model_agra_WayPointType_c* value);
void pyramid_data_model_agra_WayPointPointChoiceType_c_free(pyramid_data_model_agra_WayPointPointChoiceType_c* value);
void pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c_free(pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c* value);
void pyramid_data_model_agra_MA_PlanningAllowedEscalationType_c_free(pyramid_data_model_agra_MA_PlanningAllowedEscalationType_c* value);
void pyramid_data_model_agra_MA_MissionPlanningAutonomySettingByResultType_c_free(pyramid_data_model_agra_MA_MissionPlanningAutonomySettingByResultType_c* value);
void pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c_free(pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c* value);
void pyramid_data_model_agra_MA_PlanningAllowedType_c_free(pyramid_data_model_agra_MA_PlanningAllowedType_c* value);
void pyramid_data_model_agra_MA_AutonomousPlanCommandType_c_free(pyramid_data_model_agra_MA_AutonomousPlanCommandType_c* value);
void pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c_free(pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c* value);
void pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c_free(pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c* value);
void pyramid_data_model_agra_PlanActivationAutonomyType_c_free(pyramid_data_model_agra_PlanActivationAutonomyType_c* value);
void pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c_free(pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c* value);
void pyramid_data_model_agra_ContingencyPathAutonomyType_c_free(pyramid_data_model_agra_ContingencyPathAutonomyType_c* value);
void pyramid_data_model_agra_ContingencyPathSpacingType_c_free(pyramid_data_model_agra_ContingencyPathSpacingType_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c_free(pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c_free(pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c* value);
void pyramid_data_model_agra_CommandStatusBaseType_c_free(pyramid_data_model_agra_CommandStatusBaseType_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c_free(pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c* value);
void pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c_free(pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c* value);
void pyramid_data_model_agra_MA_ResponseMT_c_free(pyramid_data_model_agra_MA_ResponseMT_c* value);
void pyramid_data_model_agra_MA_ResponseMDT_c_free(pyramid_data_model_agra_MA_ResponseMDT_c* value);
void pyramid_data_model_agra_MA_ResponseOptionDetailsType_c_free(pyramid_data_model_agra_MA_ResponseOptionDetailsType_c* value);
void pyramid_data_model_agra_MA_RuleResponseType_c_free(pyramid_data_model_agra_MA_RuleResponseType_c* value);
void pyramid_data_model_agra_ResponseTemplateType_c_free(pyramid_data_model_agra_ResponseTemplateType_c* value);
void pyramid_data_model_agra_RequirementsTemplateID_Type_c_free(pyramid_data_model_agra_RequirementsTemplateID_Type_c* value);
void pyramid_data_model_agra_PlanInputsCoreType_c_free(pyramid_data_model_agra_PlanInputsCoreType_c* value);
void pyramid_data_model_agra_ReplanReasonType_c_free(pyramid_data_model_agra_ReplanReasonType_c* value);
void pyramid_data_model_agra_PlanningTriggerType_c_free(pyramid_data_model_agra_PlanningTriggerType_c* value);
void pyramid_data_model_agra_MissionEnvironmentConstraintType_c_free(pyramid_data_model_agra_MissionEnvironmentConstraintType_c* value);
void pyramid_data_model_agra_RequirementRiskAdjustmentType_c_free(pyramid_data_model_agra_RequirementRiskAdjustmentType_c* value);
void pyramid_data_model_agra_RequirementChoiceType_c_free(pyramid_data_model_agra_RequirementChoiceType_c* value);
void pyramid_data_model_agra_RequirementTaxonomyChoiceType_c_free(pyramid_data_model_agra_RequirementTaxonomyChoiceType_c* value);
void pyramid_data_model_agra_ResponseAlertType_c_free(pyramid_data_model_agra_ResponseAlertType_c* value);
void pyramid_data_model_agra_OperatorRecommendationType_c_free(pyramid_data_model_agra_OperatorRecommendationType_c* value);
void pyramid_data_model_agra_RequirementsTemplateType_c_free(pyramid_data_model_agra_RequirementsTemplateType_c* value);
void pyramid_data_model_agra_RequirementTemplateOptionsType_c_free(pyramid_data_model_agra_RequirementTemplateOptionsType_c* value);
void pyramid_data_model_agra_RequirementTemplateOptionType_c_free(pyramid_data_model_agra_RequirementTemplateOptionType_c* value);
void pyramid_data_model_agra_ResponseOptionType_c_free(pyramid_data_model_agra_ResponseOptionType_c* value);
void pyramid_data_model_agra_TaskResponseType_c_free(pyramid_data_model_agra_TaskResponseType_c* value);
void pyramid_data_model_agra_AirSampleTaskBaseType_c_free(pyramid_data_model_agra_AirSampleTaskBaseType_c* value);
void pyramid_data_model_agra_ProductOutputCommandBasicType_c_free(pyramid_data_model_agra_ProductOutputCommandBasicType_c* value);
void pyramid_data_model_agra_ProductOutputType_c_free(pyramid_data_model_agra_ProductOutputType_c* value);
void pyramid_data_model_agra_FileFormatType_c_free(pyramid_data_model_agra_FileFormatType_c* value);
void pyramid_data_model_agra_AMTI_TaskBaseType_c_free(pyramid_data_model_agra_AMTI_TaskBaseType_c* value);
void pyramid_data_model_agra_AMTI_CollectionConstraintsType_c_free(pyramid_data_model_agra_AMTI_CollectionConstraintsType_c* value);
void pyramid_data_model_agra_CollectionConstraintsType_c_free(pyramid_data_model_agra_CollectionConstraintsType_c* value);
void pyramid_data_model_agra_AngleQuarterPairType_c_free(pyramid_data_model_agra_AngleQuarterPairType_c* value);
void pyramid_data_model_agra_EmconConstraintType_c_free(pyramid_data_model_agra_EmconConstraintType_c* value);
void pyramid_data_model_agra_EmconOverrideType_c_free(pyramid_data_model_agra_EmconOverrideType_c* value);
void pyramid_data_model_agra_EmconERP_Type_c_free(pyramid_data_model_agra_EmconERP_Type_c* value);
void pyramid_data_model_agra_SpeedRangeType_c_free(pyramid_data_model_agra_SpeedRangeType_c* value);
void pyramid_data_model_agra_AO_TaskBaseType_c_free(pyramid_data_model_agra_AO_TaskBaseType_c* value);
void pyramid_data_model_agra_OpticalCollectionConstraintsType_c_free(pyramid_data_model_agra_OpticalCollectionConstraintsType_c* value);
void pyramid_data_model_agra_GimbalOrientationPairType_c_free(pyramid_data_model_agra_GimbalOrientationPairType_c* value);
void pyramid_data_model_agra_ComponentID_Type_c_free(pyramid_data_model_agra_ComponentID_Type_c* value);
void pyramid_data_model_agra_GimbalAxisPairType_c_free(pyramid_data_model_agra_GimbalAxisPairType_c* value);
void pyramid_data_model_agra_GimbalAxisID_Type_c_free(pyramid_data_model_agra_GimbalAxisID_Type_c* value);
void pyramid_data_model_agra_COMINT_TaskBaseType_c_free(pyramid_data_model_agra_COMINT_TaskBaseType_c* value);
void pyramid_data_model_agra_CommRelayTaskBaseType_c_free(pyramid_data_model_agra_CommRelayTaskBaseType_c* value);
void pyramid_data_model_agra_FrequencySetType_c_free(pyramid_data_model_agra_FrequencySetType_c* value);
void pyramid_data_model_agra_FrequencyMultiChannelType_c_free(pyramid_data_model_agra_FrequencyMultiChannelType_c* value);
void pyramid_data_model_agra_EA_ResponseType_c_free(pyramid_data_model_agra_EA_ResponseType_c* value);
void pyramid_data_model_agra_EA_TaskEscortType_c_free(pyramid_data_model_agra_EA_TaskEscortType_c* value);
void pyramid_data_model_agra_ProtectedAssetType_c_free(pyramid_data_model_agra_ProtectedAssetType_c* value);
void pyramid_data_model_agra_EA_TaskProtectedAssetsType_c_free(pyramid_data_model_agra_EA_TaskProtectedAssetsType_c* value);
void pyramid_data_model_agra_ProtectedAssetAndThreatType_c_free(pyramid_data_model_agra_ProtectedAssetAndThreatType_c* value);
void pyramid_data_model_agra_EA_TaskThreatsType_c_free(pyramid_data_model_agra_EA_TaskThreatsType_c* value);
void pyramid_data_model_agra_EA_TaskSuppressionConstraintsType_c_free(pyramid_data_model_agra_EA_TaskSuppressionConstraintsType_c* value);
void pyramid_data_model_agra_EA_TargetType_c_free(pyramid_data_model_agra_EA_TargetType_c* value);
void pyramid_data_model_agra_EA_EmitterDataType_c_free(pyramid_data_model_agra_EA_EmitterDataType_c* value);
void pyramid_data_model_agra_EA_TargetPointingType_c_free(pyramid_data_model_agra_EA_TargetPointingType_c* value);
void pyramid_data_model_agra_AirVolumeSensorReferencedType_c_free(pyramid_data_model_agra_AirVolumeSensorReferencedType_c* value);
void pyramid_data_model_agra_AltitudeRangePairType_c_free(pyramid_data_model_agra_AltitudeRangePairType_c* value);
void pyramid_data_model_agra_EA_PowerType_c_free(pyramid_data_model_agra_EA_PowerType_c* value);
void pyramid_data_model_agra_EA_TechniqueIdentifierType_c_free(pyramid_data_model_agra_EA_TechniqueIdentifierType_c* value);
void pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c_free(pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c* value);
void pyramid_data_model_agra_ESM_TaskBaseType_c_free(pyramid_data_model_agra_ESM_TaskBaseType_c* value);
void pyramid_data_model_agra_FlightTaskBaseType_c_free(pyramid_data_model_agra_FlightTaskBaseType_c* value);
void pyramid_data_model_agra_OrbitChangeTaskBaseType_c_free(pyramid_data_model_agra_OrbitChangeTaskBaseType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c_free(pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSearchType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSearchType_c* value);
void pyramid_data_model_agra_OrbitAccuracyType_c_free(pyramid_data_model_agra_OrbitAccuracyType_c* value);
void pyramid_data_model_agra_CharacterizationObjectiveType_c_free(pyramid_data_model_agra_CharacterizationObjectiveType_c* value);
void pyramid_data_model_agra_CharacterizationOptionsType_c_free(pyramid_data_model_agra_CharacterizationOptionsType_c* value);
void pyramid_data_model_agra_CharacterizationChoiceType_c_free(pyramid_data_model_agra_CharacterizationChoiceType_c* value);
void pyramid_data_model_agra_FrequencyParamsType_c_free(pyramid_data_model_agra_FrequencyParamsType_c* value);
void pyramid_data_model_agra_IR_ImageParamsType_c_free(pyramid_data_model_agra_IR_ImageParamsType_c* value);
void pyramid_data_model_agra_MetricParamsType_c_free(pyramid_data_model_agra_MetricParamsType_c* value);
void pyramid_data_model_agra_Narrowband_SOI_ParamsType_c_free(pyramid_data_model_agra_Narrowband_SOI_ParamsType_c* value);
void pyramid_data_model_agra_OpticalImageParamsType_c_free(pyramid_data_model_agra_OpticalImageParamsType_c* value);
void pyramid_data_model_agra_ProductResolutionType_c_free(pyramid_data_model_agra_ProductResolutionType_c* value);
void pyramid_data_model_agra_RCS_ParamsType_c_free(pyramid_data_model_agra_RCS_ParamsType_c* value);
void pyramid_data_model_agra_VisMagParamsType_c_free(pyramid_data_model_agra_VisMagParamsType_c* value);
void pyramid_data_model_agra_Wideband_SOI_ParamsType_c_free(pyramid_data_model_agra_Wideband_SOI_ParamsType_c* value);
void pyramid_data_model_agra_RangeResolutionType_c_free(pyramid_data_model_agra_RangeResolutionType_c* value);
void pyramid_data_model_agra_PhotometryParamsType_c_free(pyramid_data_model_agra_PhotometryParamsType_c* value);
void pyramid_data_model_agra_ColorPhotometryParamsType_c_free(pyramid_data_model_agra_ColorPhotometryParamsType_c* value);
void pyramid_data_model_agra_StabilityCharacterizationType_c_free(pyramid_data_model_agra_StabilityCharacterizationType_c* value);
void pyramid_data_model_agra_StructureAssessmentType_c_free(pyramid_data_model_agra_StructureAssessmentType_c* value);
void pyramid_data_model_agra_SizeEstimationType_c_free(pyramid_data_model_agra_SizeEstimationType_c* value);
void pyramid_data_model_agra_ResolvedCharacterizationType_c_free(pyramid_data_model_agra_ResolvedCharacterizationType_c* value);
void pyramid_data_model_agra_DistanceResolutionSpecificationType_c_free(pyramid_data_model_agra_DistanceResolutionSpecificationType_c* value);
void pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c_free(pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c* value);
void pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c_free(pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c* value);
void pyramid_data_model_agra_IdentificationVerificationType_c_free(pyramid_data_model_agra_IdentificationVerificationType_c* value);
void pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c_free(pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c* value);
void pyramid_data_model_agra_MultiObjectType_c_free(pyramid_data_model_agra_MultiObjectType_c* value);
void pyramid_data_model_agra_ManeuverDetectionType_c_free(pyramid_data_model_agra_ManeuverDetectionType_c* value);
void pyramid_data_model_agra_ManeuverConstraintsChoiceType_c_free(pyramid_data_model_agra_ManeuverConstraintsChoiceType_c* value);
void pyramid_data_model_agra_BasicManeuverConstraintsType_c_free(pyramid_data_model_agra_BasicManeuverConstraintsType_c* value);
void pyramid_data_model_agra_OrbitalManeuverDetailsType_c_free(pyramid_data_model_agra_OrbitalManeuverDetailsType_c* value);
void pyramid_data_model_agra_DeploymentDetectionType_c_free(pyramid_data_model_agra_DeploymentDetectionType_c* value);
void pyramid_data_model_agra_ProductNeededByType_c_free(pyramid_data_model_agra_ProductNeededByType_c* value);
void pyramid_data_model_agra_AllowableSensorsType_c_free(pyramid_data_model_agra_AllowableSensorsType_c* value);
void pyramid_data_model_agra_SensorCountConstraintType_c_free(pyramid_data_model_agra_SensorCountConstraintType_c* value);
void pyramid_data_model_agra_SensorConstraintsType_c_free(pyramid_data_model_agra_SensorConstraintsType_c* value);
void pyramid_data_model_agra_SensorConstraintsBaseType_c_free(pyramid_data_model_agra_SensorConstraintsBaseType_c* value);
void pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c_free(pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c* value);
void pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c_free(pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c* value);
void pyramid_data_model_agra_PO_TaskBaseType_c_free(pyramid_data_model_agra_PO_TaskBaseType_c* value);
void pyramid_data_model_agra_PO_ProductOutputCommandImageryType_c_free(pyramid_data_model_agra_PO_ProductOutputCommandImageryType_c* value);
void pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c_free(pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c* value);
void pyramid_data_model_agra_JPEG_SettingsType_c_free(pyramid_data_model_agra_JPEG_SettingsType_c* value);
void pyramid_data_model_agra_JPEG_WaveletTransformType_c_free(pyramid_data_model_agra_JPEG_WaveletTransformType_c* value);
void pyramid_data_model_agra_NITF_PackingPlanPET_c_free(pyramid_data_model_agra_NITF_PackingPlanPET_c* value);
void pyramid_data_model_agra_MISP_PackingPlanType_c_free(pyramid_data_model_agra_MISP_PackingPlanType_c* value);
void pyramid_data_model_agra_UMID_VideoID_Type_c_free(pyramid_data_model_agra_UMID_VideoID_Type_c* value);
void pyramid_data_model_agra_UMID_AudioID_Type_c_free(pyramid_data_model_agra_UMID_AudioID_Type_c* value);
void pyramid_data_model_agra_UMID_DataID_Type_c_free(pyramid_data_model_agra_UMID_DataID_Type_c* value);
void pyramid_data_model_agra_UMID_SystemID_Type_c_free(pyramid_data_model_agra_UMID_SystemID_Type_c* value);
void pyramid_data_model_agra_VideoOutputSettingsType_c_free(pyramid_data_model_agra_VideoOutputSettingsType_c* value);
void pyramid_data_model_agra_VideoEncoderOutputType_c_free(pyramid_data_model_agra_VideoEncoderOutputType_c* value);
void pyramid_data_model_agra_IP_ConnectionChoiceType_c_free(pyramid_data_model_agra_IP_ConnectionChoiceType_c* value);
void pyramid_data_model_agra_IPv4_ConnectionType_c_free(pyramid_data_model_agra_IPv4_ConnectionType_c* value);
void pyramid_data_model_agra_IPv6_ConnectionType_c_free(pyramid_data_model_agra_IPv6_ConnectionType_c* value);
void pyramid_data_model_agra_FileNameAndOutputType_c_free(pyramid_data_model_agra_FileNameAndOutputType_c* value);
void pyramid_data_model_agra_FileOutputType_c_free(pyramid_data_model_agra_FileOutputType_c* value);
void pyramid_data_model_agra_VideoEncoderSettingsType_c_free(pyramid_data_model_agra_VideoEncoderSettingsType_c* value);
void pyramid_data_model_agra_UnsignedIntegerMinMaxType_c_free(pyramid_data_model_agra_UnsignedIntegerMinMaxType_c* value);
void pyramid_data_model_agra_CropType_c_free(pyramid_data_model_agra_CropType_c* value);
void pyramid_data_model_agra_CropSettingsType_c_free(pyramid_data_model_agra_CropSettingsType_c* value);
void pyramid_data_model_agra_RefuelTaskBaseType_c_free(pyramid_data_model_agra_RefuelTaskBaseType_c* value);
void pyramid_data_model_agra_SAR_TaskBaseType_c_free(pyramid_data_model_agra_SAR_TaskBaseType_c* value);
void pyramid_data_model_agra_SAR_CollectionOptionsType_c_free(pyramid_data_model_agra_SAR_CollectionOptionsType_c* value);
void pyramid_data_model_agra_RadarCollectionOptionsType_c_free(pyramid_data_model_agra_RadarCollectionOptionsType_c* value);
void pyramid_data_model_agra_RadarSpoilTaperType_c_free(pyramid_data_model_agra_RadarSpoilTaperType_c* value);
void pyramid_data_model_agra_RadarTaperType_c_free(pyramid_data_model_agra_RadarTaperType_c* value);
void pyramid_data_model_agra_RadarTaperWeightingFunctionType_c_free(pyramid_data_model_agra_RadarTaperWeightingFunctionType_c* value);
void pyramid_data_model_agra_RadarSpoilType_c_free(pyramid_data_model_agra_RadarSpoilType_c* value);
void pyramid_data_model_agra_ElectronicProtectionOptionsEnableType_c_free(pyramid_data_model_agra_ElectronicProtectionOptionsEnableType_c* value);
void pyramid_data_model_agra_SupportedResolutionID_Type_c_free(pyramid_data_model_agra_SupportedResolutionID_Type_c* value);
void pyramid_data_model_agra_SAR_CollectionConstraintsType_c_free(pyramid_data_model_agra_SAR_CollectionConstraintsType_c* value);
void pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c_free(pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c* value);
void pyramid_data_model_agra_PositionLocationUncertaintyType_c_free(pyramid_data_model_agra_PositionLocationUncertaintyType_c* value);
void pyramid_data_model_agra_SAR_WaveformType_c_free(pyramid_data_model_agra_SAR_WaveformType_c* value);
void pyramid_data_model_agra_ProductOutputCommandImageryType_c_free(pyramid_data_model_agra_ProductOutputCommandImageryType_c* value);
void pyramid_data_model_agra_SMTI_TaskBaseType_c_free(pyramid_data_model_agra_SMTI_TaskBaseType_c* value);
void pyramid_data_model_agra_SMTI_CollectionOptionsType_c_free(pyramid_data_model_agra_SMTI_CollectionOptionsType_c* value);
void pyramid_data_model_agra_RangeDopplerResolutionType_c_free(pyramid_data_model_agra_RangeDopplerResolutionType_c* value);
void pyramid_data_model_agra_HRR_OptionsType_c_free(pyramid_data_model_agra_HRR_OptionsType_c* value);
void pyramid_data_model_agra_HRR_ChipSizeType_c_free(pyramid_data_model_agra_HRR_ChipSizeType_c* value);
void pyramid_data_model_agra_SMTI_CollectionConstraintsType_c_free(pyramid_data_model_agra_SMTI_CollectionConstraintsType_c* value);
void pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c_free(pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c* value);
void pyramid_data_model_agra_FalseAlarmType_c_free(pyramid_data_model_agra_FalseAlarmType_c* value);
void pyramid_data_model_agra_ProductOutputCommandSMTI_Type_c_free(pyramid_data_model_agra_ProductOutputCommandSMTI_Type_c* value);
void pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c_free(pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c* value);
void pyramid_data_model_agra_StrikeTaskWeaponListType_c_free(pyramid_data_model_agra_StrikeTaskWeaponListType_c* value);
void pyramid_data_model_agra_StrikeTaskWeaponType_c_free(pyramid_data_model_agra_StrikeTaskWeaponType_c* value);
void pyramid_data_model_agra_WeaponeeringType_c_free(pyramid_data_model_agra_WeaponeeringType_c* value);
void pyramid_data_model_agra_WeaponeeringStoreType_c_free(pyramid_data_model_agra_WeaponeeringStoreType_c* value);
void pyramid_data_model_agra_WeaponeeringTargetInfoType_c_free(pyramid_data_model_agra_WeaponeeringTargetInfoType_c* value);
void pyramid_data_model_agra_ApproachConditionsType_c_free(pyramid_data_model_agra_ApproachConditionsType_c* value);
void pyramid_data_model_agra_ApproachAngleType_c_free(pyramid_data_model_agra_ApproachAngleType_c* value);
void pyramid_data_model_agra_AzElReferenceType_c_free(pyramid_data_model_agra_AzElReferenceType_c* value);
void pyramid_data_model_agra_TargetFinalApproachType_c_free(pyramid_data_model_agra_TargetFinalApproachType_c* value);
void pyramid_data_model_agra_ImpactConditionsType_c_free(pyramid_data_model_agra_ImpactConditionsType_c* value);
void pyramid_data_model_agra_FuzeType_c_free(pyramid_data_model_agra_FuzeType_c* value);
void pyramid_data_model_agra_FuzeTriggerType_c_free(pyramid_data_model_agra_FuzeTriggerType_c* value);
void pyramid_data_model_agra_ImpactPointType_c_free(pyramid_data_model_agra_ImpactPointType_c* value);
void pyramid_data_model_agra_OffsetLocationErrorType_c_free(pyramid_data_model_agra_OffsetLocationErrorType_c* value);
void pyramid_data_model_agra_OffsetLocationType_c_free(pyramid_data_model_agra_OffsetLocationType_c* value);
void pyramid_data_model_agra_BodyFaceType_c_free(pyramid_data_model_agra_BodyFaceType_c* value);
void pyramid_data_model_agra_SystemDeploymentTaskBaseType_c_free(pyramid_data_model_agra_SystemDeploymentTaskBaseType_c* value);
void pyramid_data_model_agra_TacticalOrderTaskBaseType_c_free(pyramid_data_model_agra_TacticalOrderTaskBaseType_c* value);
void pyramid_data_model_agra_CommandResponseType_c_free(pyramid_data_model_agra_CommandResponseType_c* value);
void pyramid_data_model_agra_AirSampleCommandResponseType_c_free(pyramid_data_model_agra_AirSampleCommandResponseType_c* value);
void pyramid_data_model_agra_AMTI_CommandResponseType_c_free(pyramid_data_model_agra_AMTI_CommandResponseType_c* value);
void pyramid_data_model_agra_AO_CommandResponseType_c_free(pyramid_data_model_agra_AO_CommandResponseType_c* value);
void pyramid_data_model_agra_COMINT_CommandResponseType_c_free(pyramid_data_model_agra_COMINT_CommandResponseType_c* value);
void pyramid_data_model_agra_CommRelayCommandResponseType_c_free(pyramid_data_model_agra_CommRelayCommandResponseType_c* value);
void pyramid_data_model_agra_EA_CommandResponseType_c_free(pyramid_data_model_agra_EA_CommandResponseType_c* value);
void pyramid_data_model_agra_ESM_CommandResponseType_c_free(pyramid_data_model_agra_ESM_CommandResponseType_c* value);
void pyramid_data_model_agra_PO_CommandResponseType_c_free(pyramid_data_model_agra_PO_CommandResponseType_c* value);
void pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c_free(pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c* value);
void pyramid_data_model_agra_PO_AngleConstraintControlsType_c_free(pyramid_data_model_agra_PO_AngleConstraintControlsType_c* value);
void pyramid_data_model_agra_PO_ConstraintControlsType_c_free(pyramid_data_model_agra_PO_ConstraintControlsType_c* value);
void pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c_free(pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c* value);
void pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c_free(pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c* value);
void pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c_free(pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c* value);
void pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c_free(pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c* value);
void pyramid_data_model_agra_GimbalAxisControlType_c_free(pyramid_data_model_agra_GimbalAxisControlType_c* value);
void pyramid_data_model_agra_SAR_CommandResponseType_c_free(pyramid_data_model_agra_SAR_CommandResponseType_c* value);
void pyramid_data_model_agra_SMTI_CommandResponseType_c_free(pyramid_data_model_agra_SMTI_CommandResponseType_c* value);
void pyramid_data_model_agra_StrikeWeaponCommandType_c_free(pyramid_data_model_agra_StrikeWeaponCommandType_c* value);
void pyramid_data_model_agra_RequirementConstraintsType_c_free(pyramid_data_model_agra_RequirementConstraintsType_c* value);
void pyramid_data_model_agra_RequirementAllocationParametersType_c_free(pyramid_data_model_agra_RequirementAllocationParametersType_c* value);
void pyramid_data_model_agra_RequirementAllocationConstraintType_c_free(pyramid_data_model_agra_RequirementAllocationConstraintType_c* value);
void pyramid_data_model_agra_RequirementKinematicConstraintsType_c_free(pyramid_data_model_agra_RequirementKinematicConstraintsType_c* value);
void pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c_free(pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c* value);
void pyramid_data_model_agra_RequirementGenerationDependencyType_c_free(pyramid_data_model_agra_RequirementGenerationDependencyType_c* value);
void pyramid_data_model_agra_MA_TaskMT_c_free(pyramid_data_model_agra_MA_TaskMT_c* value);
void pyramid_data_model_agra_MA_TaskMDT_c_free(pyramid_data_model_agra_MA_TaskMDT_c* value);
void pyramid_data_model_agra_MA_TaskType_c_free(pyramid_data_model_agra_MA_TaskType_c* value);
void pyramid_data_model_agra_AirSampleTaskType_c_free(pyramid_data_model_agra_AirSampleTaskType_c* value);
void pyramid_data_model_agra_AMTI_TaskType_c_free(pyramid_data_model_agra_AMTI_TaskType_c* value);
void pyramid_data_model_agra_AMTI_TargetType_c_free(pyramid_data_model_agra_AMTI_TargetType_c* value);
void pyramid_data_model_agra_AO_TaskType_c_free(pyramid_data_model_agra_AO_TaskType_c* value);
void pyramid_data_model_agra_MA_CAP_TaskType_c_free(pyramid_data_model_agra_MA_CAP_TaskType_c* value);
void pyramid_data_model_agra_MA_OrbitType_c_free(pyramid_data_model_agra_MA_OrbitType_c* value);
void pyramid_data_model_agra_MA_OrbitShapeType_c_free(pyramid_data_model_agra_MA_OrbitShapeType_c* value);
void pyramid_data_model_agra_MA_FixOrbitType_c_free(pyramid_data_model_agra_MA_FixOrbitType_c* value);
void pyramid_data_model_agra_MA_DirectionChoiceType_c_free(pyramid_data_model_agra_MA_DirectionChoiceType_c* value);
void pyramid_data_model_agra_MA_DirectionReferenceType_c_free(pyramid_data_model_agra_MA_DirectionReferenceType_c* value);
void pyramid_data_model_agra_IntervalChoiceType_c_free(pyramid_data_model_agra_IntervalChoiceType_c* value);
void pyramid_data_model_agra_TurnGeometryChoiceType_c_free(pyramid_data_model_agra_TurnGeometryChoiceType_c* value);
void pyramid_data_model_agra_MA_CircleOrbitType_c_free(pyramid_data_model_agra_MA_CircleOrbitType_c* value);
void pyramid_data_model_agra_CircleType_c_free(pyramid_data_model_agra_CircleType_c* value);
void pyramid_data_model_agra_OrbitDurationType_c_free(pyramid_data_model_agra_OrbitDurationType_c* value);
void pyramid_data_model_agra_MA_SynchronizationChoiceType_c_free(pyramid_data_model_agra_MA_SynchronizationChoiceType_c* value);
void pyramid_data_model_agra_MA_CAPRelativePositionType_c_free(pyramid_data_model_agra_MA_CAPRelativePositionType_c* value);
void pyramid_data_model_agra_MA_CAPReferencePointType_c_free(pyramid_data_model_agra_MA_CAPReferencePointType_c* value);
void pyramid_data_model_agra_MA_CAPOffsetType_c_free(pyramid_data_model_agra_MA_CAPOffsetType_c* value);
void pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c_free(pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c* value);
void pyramid_data_model_agra_PathSegmentSpeedType_c_free(pyramid_data_model_agra_PathSegmentSpeedType_c* value);
void pyramid_data_model_agra_PathSegmentSpeedChoiceType_c_free(pyramid_data_model_agra_PathSegmentSpeedChoiceType_c* value);
void pyramid_data_model_agra_PathSegmentSpeedValueType_c_free(pyramid_data_model_agra_PathSegmentSpeedValueType_c* value);
void pyramid_data_model_agra_CargoDeliveryTaskType_c_free(pyramid_data_model_agra_CargoDeliveryTaskType_c* value);
void pyramid_data_model_agra_CargoTransitionType_c_free(pyramid_data_model_agra_CargoTransitionType_c* value);
void pyramid_data_model_agra_CargoID_Type_c_free(pyramid_data_model_agra_CargoID_Type_c* value);
void pyramid_data_model_agra_SectorType_c_free(pyramid_data_model_agra_SectorType_c* value);
void pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c_free(pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c* value);
void pyramid_data_model_agra_COMINT_TaskType_c_free(pyramid_data_model_agra_COMINT_TaskType_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c* value);
void pyramid_data_model_agra_COMINT_AcquisitionTargetType_c_free(pyramid_data_model_agra_COMINT_AcquisitionTargetType_c* value);
void pyramid_data_model_agra_COMINT_TargetType_c_free(pyramid_data_model_agra_COMINT_TargetType_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c* value);
void pyramid_data_model_agra_NED_ConeType_c_free(pyramid_data_model_agra_NED_ConeType_c* value);
void pyramid_data_model_agra_NED_LOS_Type_c_free(pyramid_data_model_agra_NED_LOS_Type_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c* value);
void pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c_free(pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c* value);
void pyramid_data_model_agra_COMINT_DataCollectCommandType_c_free(pyramid_data_model_agra_COMINT_DataCollectCommandType_c* value);
void pyramid_data_model_agra_CollectionFrequencyType_c_free(pyramid_data_model_agra_CollectionFrequencyType_c* value);
void pyramid_data_model_agra_COMINT_InteractiveType_c_free(pyramid_data_model_agra_COMINT_InteractiveType_c* value);
void pyramid_data_model_agra_CommRelayTaskType_c_free(pyramid_data_model_agra_CommRelayTaskType_c* value);
void pyramid_data_model_agra_LocationType_c_free(pyramid_data_model_agra_LocationType_c* value);
void pyramid_data_model_agra_PathType_c_free(pyramid_data_model_agra_PathType_c* value);
void pyramid_data_model_agra_LoiterType_c_free(pyramid_data_model_agra_LoiterType_c* value);
void pyramid_data_model_agra_OrbitType_c_free(pyramid_data_model_agra_OrbitType_c* value);
void pyramid_data_model_agra_HoverType_c_free(pyramid_data_model_agra_HoverType_c* value);
void pyramid_data_model_agra_PointChoice3D_Type_c_free(pyramid_data_model_agra_PointChoice3D_Type_c* value);
void pyramid_data_model_agra_Point3D_RelativeType_c_free(pyramid_data_model_agra_Point3D_RelativeType_c* value);
void pyramid_data_model_agra_CounterSpaceTaskType_c_free(pyramid_data_model_agra_CounterSpaceTaskType_c* value);
void pyramid_data_model_agra_CS_EngagementDataType_c_free(pyramid_data_model_agra_CS_EngagementDataType_c* value);
void pyramid_data_model_agra_CS_DetailDataType_c_free(pyramid_data_model_agra_CS_DetailDataType_c* value);
void pyramid_data_model_agra_CS_SubDetailDataType_c_free(pyramid_data_model_agra_CS_SubDetailDataType_c* value);
void pyramid_data_model_agra_CS_SignalType_c_free(pyramid_data_model_agra_CS_SignalType_c* value);
void pyramid_data_model_agra_MA_EscortTaskType_c_free(pyramid_data_model_agra_MA_EscortTaskType_c* value);
void pyramid_data_model_agra_MA_EscortAssetType_c_free(pyramid_data_model_agra_MA_EscortAssetType_c* value);
void pyramid_data_model_agra_MA_EscortReferenceType_c_free(pyramid_data_model_agra_MA_EscortReferenceType_c* value);
void pyramid_data_model_agra_MA_EscortFollowPathType_c_free(pyramid_data_model_agra_MA_EscortFollowPathType_c* value);
void pyramid_data_model_agra_EA_TaskType_c_free(pyramid_data_model_agra_EA_TaskType_c* value);
void pyramid_data_model_agra_EA_TaskRouteRequirementsType_c_free(pyramid_data_model_agra_EA_TaskRouteRequirementsType_c* value);
void pyramid_data_model_agra_ZoneChoiceType_c_free(pyramid_data_model_agra_ZoneChoiceType_c* value);
void pyramid_data_model_agra_VolumeChoiceType_c_free(pyramid_data_model_agra_VolumeChoiceType_c* value);
void pyramid_data_model_agra_ESM_TaskType_c_free(pyramid_data_model_agra_ESM_TaskType_c* value);
void pyramid_data_model_agra_SubCapabilityDetailsType_c_free(pyramid_data_model_agra_SubCapabilityDetailsType_c* value);
void pyramid_data_model_agra_SelectAntennaType_c_free(pyramid_data_model_agra_SelectAntennaType_c* value);
void pyramid_data_model_agra_SupportCapabilityID_Type_c_free(pyramid_data_model_agra_SupportCapabilityID_Type_c* value);
void pyramid_data_model_agra_AntennaResourceChoiceType_c_free(pyramid_data_model_agra_AntennaResourceChoiceType_c* value);
void pyramid_data_model_agra_AntennaResourceID_Type_c_free(pyramid_data_model_agra_AntennaResourceID_Type_c* value);
void pyramid_data_model_agra_ESM_TargetType_c_free(pyramid_data_model_agra_ESM_TargetType_c* value);
void pyramid_data_model_agra_ESM_LocationType_c_free(pyramid_data_model_agra_ESM_LocationType_c* value);
void pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c_free(pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c* value);
void pyramid_data_model_agra_PulseDataCollectCommandType_c_free(pyramid_data_model_agra_PulseDataCollectCommandType_c* value);
void pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c_free(pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c* value);
void pyramid_data_model_agra_MA_FlightTaskType_c_free(pyramid_data_model_agra_MA_FlightTaskType_c* value);
void pyramid_data_model_agra_MA_FlightTaskBaseType_c_free(pyramid_data_model_agra_MA_FlightTaskBaseType_c* value);
void pyramid_data_model_agra_MA_LoiterType_c_free(pyramid_data_model_agra_MA_LoiterType_c* value);
void pyramid_data_model_agra_MA_HoldType_c_free(pyramid_data_model_agra_MA_HoldType_c* value);
void pyramid_data_model_agra_MA_HoldLegSpecificationType_c_free(pyramid_data_model_agra_MA_HoldLegSpecificationType_c* value);
void pyramid_data_model_agra_MA_HoldTurnSpecificationType_c_free(pyramid_data_model_agra_MA_HoldTurnSpecificationType_c* value);
void pyramid_data_model_agra_MA_OrbitDurationType_c_free(pyramid_data_model_agra_MA_OrbitDurationType_c* value);
void pyramid_data_model_agra_MustFlyType_c_free(pyramid_data_model_agra_MustFlyType_c* value);
void pyramid_data_model_agra_MustFlyLocationType_c_free(pyramid_data_model_agra_MustFlyLocationType_c* value);
void pyramid_data_model_agra_MA_FormationType_c_free(pyramid_data_model_agra_MA_FormationType_c* value);
void pyramid_data_model_agra_MA_FormationAnchorType_c_free(pyramid_data_model_agra_MA_FormationAnchorType_c* value);
void pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c_free(pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c* value);
void pyramid_data_model_agra_MA_FlightControlModesChoiceType_c_free(pyramid_data_model_agra_MA_FlightControlModesChoiceType_c* value);
void pyramid_data_model_agra_MA_HSA_CSA_Type_c_free(pyramid_data_model_agra_MA_HSA_CSA_Type_c* value);
void pyramid_data_model_agra_MA_WaypointFollowingType_c_free(pyramid_data_model_agra_MA_WaypointFollowingType_c* value);
void pyramid_data_model_agra_MA_RouteType_c_free(pyramid_data_model_agra_MA_RouteType_c* value);
void pyramid_data_model_agra_MA_RoutePathType_c_free(pyramid_data_model_agra_MA_RoutePathType_c* value);
void pyramid_data_model_agra_MA_PathSegmentType_c_free(pyramid_data_model_agra_MA_PathSegmentType_c* value);
void pyramid_data_model_agra_MA_EndPointType_c_free(pyramid_data_model_agra_MA_EndPointType_c* value);
void pyramid_data_model_agra_TurnPointType_c_free(pyramid_data_model_agra_TurnPointType_c* value);
void pyramid_data_model_agra_MA_LoiterPointType_c_free(pyramid_data_model_agra_MA_LoiterPointType_c* value);
void pyramid_data_model_agra_CivilPathTerminatorType_c_free(pyramid_data_model_agra_CivilPathTerminatorType_c* value);
void pyramid_data_model_agra_CF_CourseToFixType_c_free(pyramid_data_model_agra_CF_CourseToFixType_c* value);
void pyramid_data_model_agra_RF_RadiusToFixType_c_free(pyramid_data_model_agra_RF_RadiusToFixType_c* value);
void pyramid_data_model_agra_ClimbType_c_free(pyramid_data_model_agra_ClimbType_c* value);
void pyramid_data_model_agra_NextPathSegmentType_c_free(pyramid_data_model_agra_NextPathSegmentType_c* value);
void pyramid_data_model_agra_ConditionalPathSegmentType_c_free(pyramid_data_model_agra_ConditionalPathSegmentType_c* value);
void pyramid_data_model_agra_PathSegmentConditionType_c_free(pyramid_data_model_agra_PathSegmentConditionType_c* value);
void pyramid_data_model_agra_SegmentCaptureType_c_free(pyramid_data_model_agra_SegmentCaptureType_c* value);
void pyramid_data_model_agra_EnduranceRemainingType_c_free(pyramid_data_model_agra_EnduranceRemainingType_c* value);
void pyramid_data_model_agra_AirfieldID_Type_c_free(pyramid_data_model_agra_AirfieldID_Type_c* value);
void pyramid_data_model_agra_RunwayID_Type_c_free(pyramid_data_model_agra_RunwayID_Type_c* value);
void pyramid_data_model_agra_MA_CurveControlType_c_free(pyramid_data_model_agra_MA_CurveControlType_c* value);
void pyramid_data_model_agra_MA_NURBS_PointType_c_free(pyramid_data_model_agra_MA_NURBS_PointType_c* value);
void pyramid_data_model_agra_MA_NURBS_ControlPointType_c_free(pyramid_data_model_agra_MA_NURBS_ControlPointType_c* value);
void pyramid_data_model_agra_MA_CurveTraversingType_c_free(pyramid_data_model_agra_MA_CurveTraversingType_c* value);
void pyramid_data_model_agra_MA_LaunchType_c_free(pyramid_data_model_agra_MA_LaunchType_c* value);
void pyramid_data_model_agra_MA_CarrierLaunchType_c_free(pyramid_data_model_agra_MA_CarrierLaunchType_c* value);
void pyramid_data_model_agra_MA_CatapultID_Type_c_free(pyramid_data_model_agra_MA_CatapultID_Type_c* value);
void pyramid_data_model_agra_MA_AirfieldTakeoffType_c_free(pyramid_data_model_agra_MA_AirfieldTakeoffType_c* value);
void pyramid_data_model_agra_MA_RecoveryType_c_free(pyramid_data_model_agra_MA_RecoveryType_c* value);
void pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c_free(pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c* value);
void pyramid_data_model_agra_MA_CarrierRecoveryType_c_free(pyramid_data_model_agra_MA_CarrierRecoveryType_c* value);
void pyramid_data_model_agra_MA_DeltaType_c_free(pyramid_data_model_agra_MA_DeltaType_c* value);
void pyramid_data_model_agra_MA_AirfieldLandType_c_free(pyramid_data_model_agra_MA_AirfieldLandType_c* value);
void pyramid_data_model_agra_MA_JettisonTaskType_c_free(pyramid_data_model_agra_MA_JettisonTaskType_c* value);
void pyramid_data_model_agra_MA_JettisonStoreSelectionType_c_free(pyramid_data_model_agra_MA_JettisonStoreSelectionType_c* value);
void pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c_free(pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c* value);
void pyramid_data_model_agra_OrbitChangeTaskType_c_free(pyramid_data_model_agra_OrbitChangeTaskType_c* value);
void pyramid_data_model_agra_OrbitChangeChoiceType_c_free(pyramid_data_model_agra_OrbitChangeChoiceType_c* value);
void pyramid_data_model_agra_COE_OrbitType_c_free(pyramid_data_model_agra_COE_OrbitType_c* value);
void pyramid_data_model_agra_OrbitalVolumeType_c_free(pyramid_data_model_agra_OrbitalVolumeType_c* value);
void pyramid_data_model_agra_RSO_ApproachType_c_free(pyramid_data_model_agra_RSO_ApproachType_c* value);
void pyramid_data_model_agra_ProximityOperationsType_c_free(pyramid_data_model_agra_ProximityOperationsType_c* value);
void pyramid_data_model_agra_ProximityOrbitChoiceType_c_free(pyramid_data_model_agra_ProximityOrbitChoiceType_c* value);
void pyramid_data_model_agra_RaceTrackOrbitType_c_free(pyramid_data_model_agra_RaceTrackOrbitType_c* value);
void pyramid_data_model_agra_DisposalOrbitType_c_free(pyramid_data_model_agra_DisposalOrbitType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceTaskType_c_free(pyramid_data_model_agra_OrbitalSurveillanceTaskType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceTargetType_c_free(pyramid_data_model_agra_OrbitalSurveillanceTargetType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c_free(pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceObjectType_c_free(pyramid_data_model_agra_OrbitalSurveillanceObjectType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceObjectBaseType_c_free(pyramid_data_model_agra_OrbitalSurveillanceObjectBaseType_c* value);
void pyramid_data_model_agra_SatelliteIdentityChoiceType_c_free(pyramid_data_model_agra_SatelliteIdentityChoiceType_c* value);
void pyramid_data_model_agra_SatelliteIdentityType_c_free(pyramid_data_model_agra_SatelliteIdentityType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c_free(pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c_free(pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorTaskBaseType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorTaskBaseType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceCollectionRequirementsType_c_free(pyramid_data_model_agra_OrbitalSurveillanceCollectionRequirementsType_c* value);
void pyramid_data_model_agra_MetricCollectionType_c_free(pyramid_data_model_agra_MetricCollectionType_c* value);
void pyramid_data_model_agra_ObservationsPerTrackLimitsType_c_free(pyramid_data_model_agra_ObservationsPerTrackLimitsType_c* value);
void pyramid_data_model_agra_SensorCharacterizationChoiceType_c_free(pyramid_data_model_agra_SensorCharacterizationChoiceType_c* value);
void pyramid_data_model_agra_StructureAssessmentCharacterizationType_c_free(pyramid_data_model_agra_StructureAssessmentCharacterizationType_c* value);
void pyramid_data_model_agra_SizeEstimationCharacterizationType_c_free(pyramid_data_model_agra_SizeEstimationCharacterizationType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c* value);
void pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c_free(pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorReportingCategoriesType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorReportingCategoriesType_c* value);
void pyramid_data_model_agra_ReportToType_c_free(pyramid_data_model_agra_ReportToType_c* value);
void pyramid_data_model_agra_AppliesToType_c_free(pyramid_data_model_agra_AppliesToType_c* value);
void pyramid_data_model_agra_ContactDetailsType_c_free(pyramid_data_model_agra_ContactDetailsType_c* value);
void pyramid_data_model_agra_OperatorNameType_c_free(pyramid_data_model_agra_OperatorNameType_c* value);
void pyramid_data_model_agra_UnitIdentityType_c_free(pyramid_data_model_agra_UnitIdentityType_c* value);
void pyramid_data_model_agra_UnitID_Type_c_free(pyramid_data_model_agra_UnitID_Type_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c* value);
void pyramid_data_model_agra_PercentileRCSType_c_free(pyramid_data_model_agra_PercentileRCSType_c* value);
void pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c_free(pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c* value);
void pyramid_data_model_agra_SensorPointListType_c_free(pyramid_data_model_agra_SensorPointListType_c* value);
void pyramid_data_model_agra_AzimuthElevationRangePointType_c_free(pyramid_data_model_agra_AzimuthElevationRangePointType_c* value);
void pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c_free(pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c* value);
void pyramid_data_model_agra_RightAscensionDeclinationPointType_c_free(pyramid_data_model_agra_RightAscensionDeclinationPointType_c* value);
void pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c_free(pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c* value);
void pyramid_data_model_agra_SensorPointListType_Point3DList_List_c_free(pyramid_data_model_agra_SensorPointListType_Point3DList_List_c* value);
void pyramid_data_model_agra_ElementSetCloudType_c_free(pyramid_data_model_agra_ElementSetCloudType_c* value);
void pyramid_data_model_agra_TLE_Type_c_free(pyramid_data_model_agra_TLE_Type_c* value);
void pyramid_data_model_agra_SourceCoverageType_c_free(pyramid_data_model_agra_SourceCoverageType_c* value);
void pyramid_data_model_agra_AngleRateRangeType_c_free(pyramid_data_model_agra_AngleRateRangeType_c* value);
void pyramid_data_model_agra_DoubleMinMaxType_c_free(pyramid_data_model_agra_DoubleMinMaxType_c* value);
void pyramid_data_model_agra_CapabilityCoverageAreaID_Type_c_free(pyramid_data_model_agra_CapabilityCoverageAreaID_Type_c* value);
void pyramid_data_model_agra_PO_TaskType_c_free(pyramid_data_model_agra_PO_TaskType_c* value);
void pyramid_data_model_agra_PointingType_c_free(pyramid_data_model_agra_PointingType_c* value);
void pyramid_data_model_agra_PointingType_Geospatial_List_c_free(pyramid_data_model_agra_PointingType_Geospatial_List_c* value);
void pyramid_data_model_agra_LOS_D_Type_c_free(pyramid_data_model_agra_LOS_D_Type_c* value);
void pyramid_data_model_agra_LOS_VariableB_Type_c_free(pyramid_data_model_agra_LOS_VariableB_Type_c* value);
void pyramid_data_model_agra_LOS_VariableA_Type_c_free(pyramid_data_model_agra_LOS_VariableA_Type_c* value);
void pyramid_data_model_agra_LOS_RatesType_c_free(pyramid_data_model_agra_LOS_RatesType_c* value);
void pyramid_data_model_agra_PO_AirTargetVolumeCommandType_c_free(pyramid_data_model_agra_PO_AirTargetVolumeCommandType_c* value);
void pyramid_data_model_agra_PO_AirTargetVolumeType_c_free(pyramid_data_model_agra_PO_AirTargetVolumeType_c* value);
void pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c_free(pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c* value);
void pyramid_data_model_agra_PointingType_Volume_List_c_free(pyramid_data_model_agra_PointingType_Volume_List_c* value);
void pyramid_data_model_agra_RefuelTaskType_c_free(pyramid_data_model_agra_RefuelTaskType_c* value);
void pyramid_data_model_agra_IdentityKindAssetType_c_free(pyramid_data_model_agra_IdentityKindAssetType_c* value);
void pyramid_data_model_agra_RequirementPlanningCandidateType_c_free(pyramid_data_model_agra_RequirementPlanningCandidateType_c* value);
void pyramid_data_model_agra_PlanningCandidateBaseType_c_free(pyramid_data_model_agra_PlanningCandidateBaseType_c* value);
void pyramid_data_model_agra_ConstrainingPlansType_c_free(pyramid_data_model_agra_ConstrainingPlansType_c* value);
void pyramid_data_model_agra_TaskPlanConstraintType_c_free(pyramid_data_model_agra_TaskPlanConstraintType_c* value);
void pyramid_data_model_agra_OtherSystemConstrainingPlansType_c_free(pyramid_data_model_agra_OtherSystemConstrainingPlansType_c* value);
void pyramid_data_model_agra_SAR_TaskType_c_free(pyramid_data_model_agra_SAR_TaskType_c* value);
void pyramid_data_model_agra_SAR_TaskTargetType_c_free(pyramid_data_model_agra_SAR_TaskTargetType_c* value);
void pyramid_data_model_agra_SAR_TargetType_c_free(pyramid_data_model_agra_SAR_TargetType_c* value);
void pyramid_data_model_agra_ISAR_TargetType_c_free(pyramid_data_model_agra_ISAR_TargetType_c* value);
void pyramid_data_model_agra_SMTI_TaskType_c_free(pyramid_data_model_agra_SMTI_TaskType_c* value);
void pyramid_data_model_agra_StrikeTaskType_c_free(pyramid_data_model_agra_StrikeTaskType_c* value);
void pyramid_data_model_agra_TargetInformationType_c_free(pyramid_data_model_agra_TargetInformationType_c* value);
void pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c_free(pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c* value);
void pyramid_data_model_agra_AreaConstraintsType_c_free(pyramid_data_model_agra_AreaConstraintsType_c* value);
void pyramid_data_model_agra_AltitudeConstraintsType_c_free(pyramid_data_model_agra_AltitudeConstraintsType_c* value);
void pyramid_data_model_agra_SystemDeploymentTaskType_c_free(pyramid_data_model_agra_SystemDeploymentTaskType_c* value);
void pyramid_data_model_agra_DeployableSystemIdentityType_c_free(pyramid_data_model_agra_DeployableSystemIdentityType_c* value);
void pyramid_data_model_agra_TacticalOrderTaskType_c_free(pyramid_data_model_agra_TacticalOrderTaskType_c* value);
void pyramid_data_model_agra_WeatherRadarTaskType_c_free(pyramid_data_model_agra_WeatherRadarTaskType_c* value);
void pyramid_data_model_agra_MA_ConstraintID_Type_c_free(pyramid_data_model_agra_MA_ConstraintID_Type_c* value);
void pyramid_data_model_agra_MA_TaskStatusMT_c_free(pyramid_data_model_agra_MA_TaskStatusMT_c* value);
void pyramid_data_model_agra_MA_TaskStatusMDT_c_free(pyramid_data_model_agra_MA_TaskStatusMDT_c* value);
void pyramid_data_model_agra_MissionContingencyAlertMT_c_free(pyramid_data_model_agra_MissionContingencyAlertMT_c* value);
void pyramid_data_model_agra_MissionContingencyAlertMDT_c_free(pyramid_data_model_agra_MissionContingencyAlertMDT_c* value);
void pyramid_data_model_agra_MissionContingencyConditionType_c_free(pyramid_data_model_agra_MissionContingencyConditionType_c* value);
void pyramid_data_model_agra_ConflictType_c_free(pyramid_data_model_agra_ConflictType_c* value);
void pyramid_data_model_agra_ConflictLocationType_c_free(pyramid_data_model_agra_ConflictLocationType_c* value);
void pyramid_data_model_agra_RoutePlanSegmentReferenceType_c_free(pyramid_data_model_agra_RoutePlanSegmentReferenceType_c* value);
void pyramid_data_model_agra_RequirementsReferenceType_c_free(pyramid_data_model_agra_RequirementsReferenceType_c* value);
void pyramid_data_model_agra_RequirementsReferenceLockableType_c_free(pyramid_data_model_agra_RequirementsReferenceLockableType_c* value);
void pyramid_data_model_agra_AutonomousActionStatusChoiceType_c_free(pyramid_data_model_agra_AutonomousActionStatusChoiceType_c* value);
void pyramid_data_model_agra_AutonomousPlanningActionStatusType_c_free(pyramid_data_model_agra_AutonomousPlanningActionStatusType_c* value);
void pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c_free(pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c* value);

#ifdef __cplusplus
}
#endif

#endif /* PYRAMID_DATA_MODEL_AGRA_CABI_H */
