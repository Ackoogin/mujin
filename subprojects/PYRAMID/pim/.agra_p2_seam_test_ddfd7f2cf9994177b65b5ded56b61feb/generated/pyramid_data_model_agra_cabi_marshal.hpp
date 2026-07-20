#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_agra_cabi.h"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::agra::MA_ActionMT& in, pyramid_data_model_agra_MA_ActionMT_c* out);
void from_c(const pyramid_data_model_agra_MA_ActionMT_c* in, pyramid::domain_model::agra::MA_ActionMT& out);

void to_c(const pyramid::domain_model::agra::MessageType& in, pyramid_data_model_agra_MessageType_c* out);
void from_c(const pyramid_data_model_agra_MessageType_c* in, pyramid::domain_model::agra::MessageType& out);

void to_c(const pyramid::domain_model::agra::SecurityInformationType& in, pyramid_data_model_agra_SecurityInformationType_c* out);
void from_c(const pyramid_data_model_agra_SecurityInformationType_c* in, pyramid::domain_model::agra::SecurityInformationType& out);

void to_c(const pyramid::domain_model::agra::OwnerProducerChoiceType& in, pyramid_data_model_agra_OwnerProducerChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OwnerProducerChoiceType_c* in, pyramid::domain_model::agra::OwnerProducerChoiceType& out);

void to_c(const pyramid::domain_model::agra::SCI_ControlsChoiceType& in, pyramid_data_model_agra_SCI_ControlsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_SCI_ControlsChoiceType_c* in, pyramid::domain_model::agra::SCI_ControlsChoiceType& out);

void to_c(const pyramid::domain_model::agra::AtomicEnergyMarkingsChoiceType& in, pyramid_data_model_agra_AtomicEnergyMarkingsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_AtomicEnergyMarkingsChoiceType_c* in, pyramid::domain_model::agra::AtomicEnergyMarkingsChoiceType& out);

void to_c(const pyramid::domain_model::agra::ReleasableToChoiceType& in, pyramid_data_model_agra_ReleasableToChoiceType_c* out);
void from_c(const pyramid_data_model_agra_ReleasableToChoiceType_c* in, pyramid::domain_model::agra::ReleasableToChoiceType& out);

void to_c(const pyramid::domain_model::agra::FGI_SourceOpenChoiceType& in, pyramid_data_model_agra_FGI_SourceOpenChoiceType_c* out);
void from_c(const pyramid_data_model_agra_FGI_SourceOpenChoiceType_c* in, pyramid::domain_model::agra::FGI_SourceOpenChoiceType& out);

void to_c(const pyramid::domain_model::agra::NonIC_MarkingsChoiceType& in, pyramid_data_model_agra_NonIC_MarkingsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_NonIC_MarkingsChoiceType_c* in, pyramid::domain_model::agra::NonIC_MarkingsChoiceType& out);

void to_c(const pyramid::domain_model::agra::HeaderType& in, pyramid_data_model_agra_HeaderType_c* out);
void from_c(const pyramid_data_model_agra_HeaderType_c* in, pyramid::domain_model::agra::HeaderType& out);

void to_c(const pyramid::domain_model::agra::SystemID_Type& in, pyramid_data_model_agra_SystemID_Type_c* out);
void from_c(const pyramid_data_model_agra_SystemID_Type_c* in, pyramid::domain_model::agra::SystemID_Type& out);

void to_c(const pyramid::domain_model::agra::ID_Type& in, pyramid_data_model_agra_ID_Type_c* out);
void from_c(const pyramid_data_model_agra_ID_Type_c* in, pyramid::domain_model::agra::ID_Type& out);

void to_c(const pyramid::domain_model::agra::ServiceID_Type& in, pyramid_data_model_agra_ServiceID_Type_c* out);
void from_c(const pyramid_data_model_agra_ServiceID_Type_c* in, pyramid::domain_model::agra::ServiceID_Type& out);

void to_c(const pyramid::domain_model::agra::MissionID_Type& in, pyramid_data_model_agra_MissionID_Type_c* out);
void from_c(const pyramid_data_model_agra_MissionID_Type_c* in, pyramid::domain_model::agra::MissionID_Type& out);

void to_c(const pyramid::domain_model::agra::VersionedID_Type& in, pyramid_data_model_agra_VersionedID_Type_c* out);
void from_c(const pyramid_data_model_agra_VersionedID_Type_c* in, pyramid::domain_model::agra::VersionedID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_ActionMDT& in, pyramid_data_model_agra_MA_ActionMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_ActionMDT_c* in, pyramid::domain_model::agra::MA_ActionMDT& out);

void to_c(const pyramid::domain_model::agra::ActionID_Type& in, pyramid_data_model_agra_ActionID_Type_c* out);
void from_c(const pyramid_data_model_agra_ActionID_Type_c* in, pyramid::domain_model::agra::ActionID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementConstraintsType& in, pyramid_data_model_agra_MA_RequirementConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementConstraintsType_c* in, pyramid::domain_model::agra::MA_RequirementConstraintsType& out);

void to_c(const pyramid::domain_model::agra::ComparableRankingType& in, pyramid_data_model_agra_ComparableRankingType_c* out);
void from_c(const pyramid_data_model_agra_ComparableRankingType_c* in, pyramid::domain_model::agra::ComparableRankingType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementAllocationParametersType& in, pyramid_data_model_agra_MA_RequirementAllocationParametersType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementAllocationParametersType_c* in, pyramid::domain_model::agra::MA_RequirementAllocationParametersType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementAllocationConstraintType& in, pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementAllocationConstraintType_c* in, pyramid::domain_model::agra::MA_RequirementAllocationConstraintType& out);

void to_c(const pyramid::domain_model::agra::MA_AllocationChoiceType& in, pyramid_data_model_agra_MA_AllocationChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_AllocationChoiceType_c* in, pyramid::domain_model::agra::MA_AllocationChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_AllocationChoiceType_SystemID_List& in, pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c* out);
void from_c(const pyramid_data_model_agra_MA_AllocationChoiceType_SystemID_List_c* in, pyramid::domain_model::agra::MA_AllocationChoiceType_SystemID_List& out);

void to_c(const pyramid::domain_model::agra::PackageID_Type& in, pyramid_data_model_agra_PackageID_Type_c* out);
void from_c(const pyramid_data_model_agra_PackageID_Type_c* in, pyramid::domain_model::agra::PackageID_Type& out);

void to_c(const pyramid::domain_model::agra::IdentityType& in, pyramid_data_model_agra_IdentityType_c* out);
void from_c(const pyramid_data_model_agra_IdentityType_c* in, pyramid::domain_model::agra::IdentityType& out);

void to_c(const pyramid::domain_model::agra::StandardIdentityType& in, pyramid_data_model_agra_StandardIdentityType_c* out);
void from_c(const pyramid_data_model_agra_StandardIdentityType_c* in, pyramid::domain_model::agra::StandardIdentityType& out);

void to_c(const pyramid::domain_model::agra::CountryCodeType& in, pyramid_data_model_agra_CountryCodeType_c* out);
void from_c(const pyramid_data_model_agra_CountryCodeType_c* in, pyramid::domain_model::agra::CountryCodeType& out);

void to_c(const pyramid::domain_model::agra::EnvironmentIdentityType& in, pyramid_data_model_agra_EnvironmentIdentityType_c* out);
void from_c(const pyramid_data_model_agra_EnvironmentIdentityType_c* in, pyramid::domain_model::agra::EnvironmentIdentityType& out);

void to_c(const pyramid::domain_model::agra::OrbitRegimeType& in, pyramid_data_model_agra_OrbitRegimeType_c* out);
void from_c(const pyramid_data_model_agra_OrbitRegimeType_c* in, pyramid::domain_model::agra::OrbitRegimeType& out);

void to_c(const pyramid::domain_model::agra::PlatformIdentityType& in, pyramid_data_model_agra_PlatformIdentityType_c* out);
void from_c(const pyramid_data_model_agra_PlatformIdentityType_c* in, pyramid::domain_model::agra::PlatformIdentityType& out);

void to_c(const pyramid::domain_model::agra::SpecificIdentityType& in, pyramid_data_model_agra_SpecificIdentityType_c* out);
void from_c(const pyramid_data_model_agra_SpecificIdentityType_c* in, pyramid::domain_model::agra::SpecificIdentityType& out);

void to_c(const pyramid::domain_model::agra::EmitterIdentityType& in, pyramid_data_model_agra_EmitterIdentityType_c* out);
void from_c(const pyramid_data_model_agra_EmitterIdentityType_c* in, pyramid::domain_model::agra::EmitterIdentityType& out);

void to_c(const pyramid::domain_model::agra::EmitterIdentityCategoryType& in, pyramid_data_model_agra_EmitterIdentityCategoryType_c* out);
void from_c(const pyramid_data_model_agra_EmitterIdentityCategoryType_c* in, pyramid::domain_model::agra::EmitterIdentityCategoryType& out);

void to_c(const pyramid::domain_model::agra::RadarEmitterIdentityType& in, pyramid_data_model_agra_RadarEmitterIdentityType_c* out);
void from_c(const pyramid_data_model_agra_RadarEmitterIdentityType_c* in, pyramid::domain_model::agra::RadarEmitterIdentityType& out);

void to_c(const pyramid::domain_model::agra::ForeignKeyType& in, pyramid_data_model_agra_ForeignKeyType_c* out);
void from_c(const pyramid_data_model_agra_ForeignKeyType_c* in, pyramid::domain_model::agra::ForeignKeyType& out);

void to_c(const pyramid::domain_model::agra::CommunicationsEmitterIdentityType& in, pyramid_data_model_agra_CommunicationsEmitterIdentityType_c* out);
void from_c(const pyramid_data_model_agra_CommunicationsEmitterIdentityType_c* in, pyramid::domain_model::agra::CommunicationsEmitterIdentityType& out);

void to_c(const pyramid::domain_model::agra::JammerEmitterIdentityType& in, pyramid_data_model_agra_JammerEmitterIdentityType_c* out);
void from_c(const pyramid_data_model_agra_JammerEmitterIdentityType_c* in, pyramid::domain_model::agra::JammerEmitterIdentityType& out);

void to_c(const pyramid::domain_model::agra::MissileEmitterIdentityType& in, pyramid_data_model_agra_MissileEmitterIdentityType_c* out);
void from_c(const pyramid_data_model_agra_MissileEmitterIdentityType_c* in, pyramid::domain_model::agra::MissileEmitterIdentityType& out);

void to_c(const pyramid::domain_model::agra::SpecificEmitterIdentityType& in, pyramid_data_model_agra_SpecificEmitterIdentityType_c* out);
void from_c(const pyramid_data_model_agra_SpecificEmitterIdentityType_c* in, pyramid::domain_model::agra::SpecificEmitterIdentityType& out);

void to_c(const pyramid::domain_model::agra::FacilityIdentificationType& in, pyramid_data_model_agra_FacilityIdentificationType_c* out);
void from_c(const pyramid_data_model_agra_FacilityIdentificationType_c* in, pyramid::domain_model::agra::FacilityIdentificationType& out);

void to_c(const pyramid::domain_model::agra::VehicleIdentificationType& in, pyramid_data_model_agra_VehicleIdentificationType_c* out);
void from_c(const pyramid_data_model_agra_VehicleIdentificationType_c* in, pyramid::domain_model::agra::VehicleIdentificationType& out);

void to_c(const pyramid::domain_model::agra::VehicleUniqueIdentifierType& in, pyramid_data_model_agra_VehicleUniqueIdentifierType_c* out);
void from_c(const pyramid_data_model_agra_VehicleUniqueIdentifierType_c* in, pyramid::domain_model::agra::VehicleUniqueIdentifierType& out);

void to_c(const pyramid::domain_model::agra::AIS_Type& in, pyramid_data_model_agra_AIS_Type_c* out);
void from_c(const pyramid_data_model_agra_AIS_Type_c* in, pyramid::domain_model::agra::AIS_Type& out);

void to_c(const pyramid::domain_model::agra::SatelliteIdentifierType& in, pyramid_data_model_agra_SatelliteIdentifierType_c* out);
void from_c(const pyramid_data_model_agra_SatelliteIdentifierType_c* in, pyramid::domain_model::agra::SatelliteIdentifierType& out);

void to_c(const pyramid::domain_model::agra::InternationalDesignatorType& in, pyramid_data_model_agra_InternationalDesignatorType_c* out);
void from_c(const pyramid_data_model_agra_InternationalDesignatorType_c* in, pyramid::domain_model::agra::InternationalDesignatorType& out);

void to_c(const pyramid::domain_model::agra::MissionInformationType& in, pyramid_data_model_agra_MissionInformationType_c* out);
void from_c(const pyramid_data_model_agra_MissionInformationType_c* in, pyramid::domain_model::agra::MissionInformationType& out);

void to_c(const pyramid::domain_model::agra::IFF_Type& in, pyramid_data_model_agra_IFF_Type_c* out);
void from_c(const pyramid_data_model_agra_IFF_Type_c* in, pyramid::domain_model::agra::IFF_Type& out);

void to_c(const pyramid::domain_model::agra::IFF_Mode1Type& in, pyramid_data_model_agra_IFF_Mode1Type_c* out);
void from_c(const pyramid_data_model_agra_IFF_Mode1Type_c* in, pyramid::domain_model::agra::IFF_Mode1Type& out);

void to_c(const pyramid::domain_model::agra::IFF_OctalModeType& in, pyramid_data_model_agra_IFF_OctalModeType_c* out);
void from_c(const pyramid_data_model_agra_IFF_OctalModeType_c* in, pyramid::domain_model::agra::IFF_OctalModeType& out);

void to_c(const pyramid::domain_model::agra::IFF_Mode4Type& in, pyramid_data_model_agra_IFF_Mode4Type_c* out);
void from_c(const pyramid_data_model_agra_IFF_Mode4Type_c* in, pyramid::domain_model::agra::IFF_Mode4Type& out);

void to_c(const pyramid::domain_model::agra::IFF_Mode5Type& in, pyramid_data_model_agra_IFF_Mode5Type_c* out);
void from_c(const pyramid_data_model_agra_IFF_Mode5Type_c* in, pyramid::domain_model::agra::IFF_Mode5Type& out);

void to_c(const pyramid::domain_model::agra::IFF_ModeS_Type& in, pyramid_data_model_agra_IFF_ModeS_Type_c* out);
void from_c(const pyramid_data_model_agra_IFF_ModeS_Type_c* in, pyramid::domain_model::agra::IFF_ModeS_Type& out);

void to_c(const pyramid::domain_model::agra::CallSignType& in, pyramid_data_model_agra_CallSignType_c* out);
void from_c(const pyramid_data_model_agra_CallSignType_c* in, pyramid::domain_model::agra::CallSignType& out);

void to_c(const pyramid::domain_model::agra::DataLinkIdentifierPET& in, pyramid_data_model_agra_DataLinkIdentifierPET_c* out);
void from_c(const pyramid_data_model_agra_DataLinkIdentifierPET_c* in, pyramid::domain_model::agra::DataLinkIdentifierPET& out);

void to_c(const pyramid::domain_model::agra::EOB_IdentityType& in, pyramid_data_model_agra_EOB_IdentityType_c* out);
void from_c(const pyramid_data_model_agra_EOB_IdentityType_c* in, pyramid::domain_model::agra::EOB_IdentityType& out);

void to_c(const pyramid::domain_model::agra::EOB_SiteIdentityType& in, pyramid_data_model_agra_EOB_SiteIdentityType_c* out);
void from_c(const pyramid_data_model_agra_EOB_SiteIdentityType_c* in, pyramid::domain_model::agra::EOB_SiteIdentityType& out);

void to_c(const pyramid::domain_model::agra::EOB_IdentityBaseType& in, pyramid_data_model_agra_EOB_IdentityBaseType_c* out);
void from_c(const pyramid_data_model_agra_EOB_IdentityBaseType_c* in, pyramid::domain_model::agra::EOB_IdentityBaseType& out);

void to_c(const pyramid::domain_model::agra::EOB_SitePIN_Type& in, pyramid_data_model_agra_EOB_SitePIN_Type_c* out);
void from_c(const pyramid_data_model_agra_EOB_SitePIN_Type_c* in, pyramid::domain_model::agra::EOB_SitePIN_Type& out);

void to_c(const pyramid::domain_model::agra::BasicEncyclopediaNumberType& in, pyramid_data_model_agra_BasicEncyclopediaNumberType_c* out);
void from_c(const pyramid_data_model_agra_BasicEncyclopediaNumberType_c* in, pyramid::domain_model::agra::BasicEncyclopediaNumberType& out);

void to_c(const pyramid::domain_model::agra::EOB_EquipmentIdentityType& in, pyramid_data_model_agra_EOB_EquipmentIdentityType_c* out);
void from_c(const pyramid_data_model_agra_EOB_EquipmentIdentityType_c* in, pyramid::domain_model::agra::EOB_EquipmentIdentityType& out);

void to_c(const pyramid::domain_model::agra::EOB_EquipmentType& in, pyramid_data_model_agra_EOB_EquipmentType_c* out);
void from_c(const pyramid_data_model_agra_EOB_EquipmentType_c* in, pyramid::domain_model::agra::EOB_EquipmentType& out);

void to_c(const pyramid::domain_model::agra::StoreType& in, pyramid_data_model_agra_StoreType_c* out);
void from_c(const pyramid_data_model_agra_StoreType_c* in, pyramid::domain_model::agra::StoreType& out);

void to_c(const pyramid::domain_model::agra::CapabilityID_Type& in, pyramid_data_model_agra_CapabilityID_Type_c* out);
void from_c(const pyramid_data_model_agra_CapabilityID_Type_c* in, pyramid::domain_model::agra::CapabilityID_Type& out);

void to_c(const pyramid::domain_model::agra::RequirementTimingType& in, pyramid_data_model_agra_RequirementTimingType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTimingType_c* in, pyramid::domain_model::agra::RequirementTimingType& out);

void to_c(const pyramid::domain_model::agra::TimingConstraintsType& in, pyramid_data_model_agra_TimingConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_TimingConstraintsType_c* in, pyramid::domain_model::agra::TimingConstraintsType& out);

void to_c(const pyramid::domain_model::agra::TimeWindowType& in, pyramid_data_model_agra_TimeWindowType_c* out);
void from_c(const pyramid_data_model_agra_TimeWindowType_c* in, pyramid::domain_model::agra::TimeWindowType& out);

void to_c(const pyramid::domain_model::agra::DateTimeRangeType& in, pyramid_data_model_agra_DateTimeRangeType_c* out);
void from_c(const pyramid_data_model_agra_DateTimeRangeType_c* in, pyramid::domain_model::agra::DateTimeRangeType& out);

void to_c(const pyramid::domain_model::agra::WeekdayIntervalType& in, pyramid_data_model_agra_WeekdayIntervalType_c* out);
void from_c(const pyramid_data_model_agra_WeekdayIntervalType_c* in, pyramid::domain_model::agra::WeekdayIntervalType& out);

void to_c(const pyramid::domain_model::agra::RepetitionConstraintsType& in, pyramid_data_model_agra_RepetitionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionConstraintsType_c* in, pyramid::domain_model::agra::RepetitionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::RepetitionType& in, pyramid_data_model_agra_RepetitionType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionType_c* in, pyramid::domain_model::agra::RepetitionType& out);

void to_c(const pyramid::domain_model::agra::RepetitionTimeBasedType& in, pyramid_data_model_agra_RepetitionTimeBasedType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionTimeBasedType_c* in, pyramid::domain_model::agra::RepetitionTimeBasedType& out);

void to_c(const pyramid::domain_model::agra::RepetitionContinuousType& in, pyramid_data_model_agra_RepetitionContinuousType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionContinuousType_c* in, pyramid::domain_model::agra::RepetitionContinuousType& out);

void to_c(const pyramid::domain_model::agra::RepetitionFiniteType& in, pyramid_data_model_agra_RepetitionFiniteType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionFiniteType_c* in, pyramid::domain_model::agra::RepetitionFiniteType& out);

void to_c(const pyramid::domain_model::agra::RepetitionPeriodicType& in, pyramid_data_model_agra_RepetitionPeriodicType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionPeriodicType_c* in, pyramid::domain_model::agra::RepetitionPeriodicType& out);

void to_c(const pyramid::domain_model::agra::RepetitionEventBasedType& in, pyramid_data_model_agra_RepetitionEventBasedType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionEventBasedType_c* in, pyramid::domain_model::agra::RepetitionEventBasedType& out);

void to_c(const pyramid::domain_model::agra::RepetitionEventType& in, pyramid_data_model_agra_RepetitionEventType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionEventType_c* in, pyramid::domain_model::agra::RepetitionEventType& out);

void to_c(const pyramid::domain_model::agra::RepetitionPositionChangeType& in, pyramid_data_model_agra_RepetitionPositionChangeType_c* out);
void from_c(const pyramid_data_model_agra_RepetitionPositionChangeType_c* in, pyramid::domain_model::agra::RepetitionPositionChangeType& out);

void to_c(const pyramid::domain_model::agra::LOS_Type& in, pyramid_data_model_agra_LOS_Type_c* out);
void from_c(const pyramid_data_model_agra_LOS_Type_c* in, pyramid::domain_model::agra::LOS_Type& out);

void to_c(const pyramid::domain_model::agra::LOS_InertialA_Type& in, pyramid_data_model_agra_LOS_InertialA_Type_c* out);
void from_c(const pyramid_data_model_agra_LOS_InertialA_Type_c* in, pyramid::domain_model::agra::LOS_InertialA_Type& out);

void to_c(const pyramid::domain_model::agra::ThresholdOffOrbitTriggerDataType& in, pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c* out);
void from_c(const pyramid_data_model_agra_ThresholdOffOrbitTriggerDataType_c* in, pyramid::domain_model::agra::ThresholdOffOrbitTriggerDataType& out);

void to_c(const pyramid::domain_model::agra::RTN_PositionDeltaType& in, pyramid_data_model_agra_RTN_PositionDeltaType_c* out);
void from_c(const pyramid_data_model_agra_RTN_PositionDeltaType_c* in, pyramid::domain_model::agra::RTN_PositionDeltaType& out);

void to_c(const pyramid::domain_model::agra::RTN_VelocityDeltaType& in, pyramid_data_model_agra_RTN_VelocityDeltaType_c* out);
void from_c(const pyramid_data_model_agra_RTN_VelocityDeltaType_c* in, pyramid::domain_model::agra::RTN_VelocityDeltaType& out);

void to_c(const pyramid::domain_model::agra::TimeErrorType& in, pyramid_data_model_agra_TimeErrorType_c* out);
void from_c(const pyramid_data_model_agra_TimeErrorType_c* in, pyramid::domain_model::agra::TimeErrorType& out);

void to_c(const pyramid::domain_model::agra::EventOffsetChoiceType& in, pyramid_data_model_agra_EventOffsetChoiceType_c* out);
void from_c(const pyramid_data_model_agra_EventOffsetChoiceType_c* in, pyramid::domain_model::agra::EventOffsetChoiceType& out);

void to_c(const pyramid::domain_model::agra::LOS_InertialB_Type& in, pyramid_data_model_agra_LOS_InertialB_Type_c* out);
void from_c(const pyramid_data_model_agra_LOS_InertialB_Type_c* in, pyramid::domain_model::agra::LOS_InertialB_Type& out);

void to_c(const pyramid::domain_model::agra::EventWindowChoiceType& in, pyramid_data_model_agra_EventWindowChoiceType_c* out);
void from_c(const pyramid_data_model_agra_EventWindowChoiceType_c* in, pyramid::domain_model::agra::EventWindowChoiceType& out);

void to_c(const pyramid::domain_model::agra::ReferenceAssetKinematicsType& in, pyramid_data_model_agra_ReferenceAssetKinematicsType_c* out);
void from_c(const pyramid_data_model_agra_ReferenceAssetKinematicsType_c* in, pyramid::domain_model::agra::ReferenceAssetKinematicsType& out);

void to_c(const pyramid::domain_model::agra::AssetType& in, pyramid_data_model_agra_AssetType_c* out);
void from_c(const pyramid_data_model_agra_AssetType_c* in, pyramid::domain_model::agra::AssetType& out);

void to_c(const pyramid::domain_model::agra::EntityID_Type& in, pyramid_data_model_agra_EntityID_Type_c* out);
void from_c(const pyramid_data_model_agra_EntityID_Type_c* in, pyramid::domain_model::agra::EntityID_Type& out);

void to_c(const pyramid::domain_model::agra::OrbitKinematicsType& in, pyramid_data_model_agra_OrbitKinematicsType_c* out);
void from_c(const pyramid_data_model_agra_OrbitKinematicsType_c* in, pyramid::domain_model::agra::OrbitKinematicsType& out);

void to_c(const pyramid::domain_model::agra::TLE_BaseType& in, pyramid_data_model_agra_TLE_BaseType_c* out);
void from_c(const pyramid_data_model_agra_TLE_BaseType_c* in, pyramid::domain_model::agra::TLE_BaseType& out);

void to_c(const pyramid::domain_model::agra::COE_OrbitBaseType& in, pyramid_data_model_agra_COE_OrbitBaseType_c* out);
void from_c(const pyramid_data_model_agra_COE_OrbitBaseType_c* in, pyramid::domain_model::agra::COE_OrbitBaseType& out);

void to_c(const pyramid::domain_model::agra::COE_OrientationType& in, pyramid_data_model_agra_COE_OrientationType_c* out);
void from_c(const pyramid_data_model_agra_COE_OrientationType_c* in, pyramid::domain_model::agra::COE_OrientationType& out);

void to_c(const pyramid::domain_model::agra::COE_NonEquatorialOrientationType& in, pyramid_data_model_agra_COE_NonEquatorialOrientationType_c* out);
void from_c(const pyramid_data_model_agra_COE_NonEquatorialOrientationType_c* in, pyramid::domain_model::agra::COE_NonEquatorialOrientationType& out);

void to_c(const pyramid::domain_model::agra::COE_EquatorialOrientationType& in, pyramid_data_model_agra_COE_EquatorialOrientationType_c* out);
void from_c(const pyramid_data_model_agra_COE_EquatorialOrientationType_c* in, pyramid::domain_model::agra::COE_EquatorialOrientationType& out);

void to_c(const pyramid::domain_model::agra::COE_PositionType& in, pyramid_data_model_agra_COE_PositionType_c* out);
void from_c(const pyramid_data_model_agra_COE_PositionType_c* in, pyramid::domain_model::agra::COE_PositionType& out);

void to_c(const pyramid::domain_model::agra::OrbitalEphemerisChoiceType& in, pyramid_data_model_agra_OrbitalEphemerisChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalEphemerisChoiceType_c* in, pyramid::domain_model::agra::OrbitalEphemerisChoiceType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType& in, pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_c* in, pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType& out);

void to_c(const pyramid::domain_model::agra::J2K_KinematicsType& in, pyramid_data_model_agra_J2K_KinematicsType_c* out);
void from_c(const pyramid_data_model_agra_J2K_KinematicsType_c* in, pyramid::domain_model::agra::J2K_KinematicsType& out);

void to_c(const pyramid::domain_model::agra::J2K_PositionType& in, pyramid_data_model_agra_J2K_PositionType_c* out);
void from_c(const pyramid_data_model_agra_J2K_PositionType_c* in, pyramid::domain_model::agra::J2K_PositionType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsVelocityType& in, pyramid_data_model_agra_OrbitalKinematicsVelocityType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsVelocityType_c* in, pyramid::domain_model::agra::OrbitalKinematicsVelocityType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsAccelerationType& in, pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsAccelerationType_c* in, pyramid::domain_model::agra::OrbitalKinematicsAccelerationType& out);

void to_c(const pyramid::domain_model::agra::QuaternionType& in, pyramid_data_model_agra_QuaternionType_c* out);
void from_c(const pyramid_data_model_agra_QuaternionType_c* in, pyramid::domain_model::agra::QuaternionType& out);

void to_c(const pyramid::domain_model::agra::CovarianceMatrixType& in, pyramid_data_model_agra_CovarianceMatrixType_c* out);
void from_c(const pyramid_data_model_agra_CovarianceMatrixType_c* in, pyramid::domain_model::agra::CovarianceMatrixType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List& in, pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List_c* in, pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType_J2K_StateVector_List& out);

void to_c(const pyramid::domain_model::agra::GCRS_KinematicsType& in, pyramid_data_model_agra_GCRS_KinematicsType_c* out);
void from_c(const pyramid_data_model_agra_GCRS_KinematicsType_c* in, pyramid::domain_model::agra::GCRS_KinematicsType& out);

void to_c(const pyramid::domain_model::agra::GCRS_PositionType& in, pyramid_data_model_agra_GCRS_PositionType_c* out);
void from_c(const pyramid_data_model_agra_GCRS_PositionType_c* in, pyramid::domain_model::agra::GCRS_PositionType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List& in, pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List_c* in, pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType_GCRS_StateVector_List& out);

void to_c(const pyramid::domain_model::agra::BCRS_KinematicsType& in, pyramid_data_model_agra_BCRS_KinematicsType_c* out);
void from_c(const pyramid_data_model_agra_BCRS_KinematicsType_c* in, pyramid::domain_model::agra::BCRS_KinematicsType& out);

void to_c(const pyramid::domain_model::agra::BCRS_PositionType& in, pyramid_data_model_agra_BCRS_PositionType_c* out);
void from_c(const pyramid_data_model_agra_BCRS_PositionType_c* in, pyramid::domain_model::agra::BCRS_PositionType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List& in, pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List_c* in, pyramid::domain_model::agra::OrbitalKinematicsStandardEphemerisType_BCRS_StateVector_List& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsRelativeEphemerisType& in, pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsRelativeEphemerisType_c* in, pyramid::domain_model::agra::OrbitalKinematicsRelativeEphemerisType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsRelativeStateVectorType& in, pyramid_data_model_agra_OrbitalKinematicsRelativeStateVectorType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsRelativeStateVectorType_c* in, pyramid::domain_model::agra::OrbitalKinematicsRelativeStateVectorType& out);

void to_c(const pyramid::domain_model::agra::RTN_KinematicsType& in, pyramid_data_model_agra_RTN_KinematicsType_c* out);
void from_c(const pyramid_data_model_agra_RTN_KinematicsType_c* in, pyramid::domain_model::agra::RTN_KinematicsType& out);

void to_c(const pyramid::domain_model::agra::RTN_PositionType& in, pyramid_data_model_agra_RTN_PositionType_c* out);
void from_c(const pyramid_data_model_agra_RTN_PositionType_c* in, pyramid::domain_model::agra::RTN_PositionType& out);

void to_c(const pyramid::domain_model::agra::RTN_VelocityType& in, pyramid_data_model_agra_RTN_VelocityType_c* out);
void from_c(const pyramid_data_model_agra_RTN_VelocityType_c* in, pyramid::domain_model::agra::RTN_VelocityType& out);

void to_c(const pyramid::domain_model::agra::RTN_AccelerationType& in, pyramid_data_model_agra_RTN_AccelerationType_c* out);
void from_c(const pyramid_data_model_agra_RTN_AccelerationType_c* in, pyramid::domain_model::agra::RTN_AccelerationType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsStandardFrameChoiceType& in, pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsStandardFrameChoiceType_c* in, pyramid::domain_model::agra::OrbitalKinematicsStandardFrameChoiceType& out);

void to_c(const pyramid::domain_model::agra::OrbitalVCM_Type& in, pyramid_data_model_agra_OrbitalVCM_Type_c* out);
void from_c(const pyramid_data_model_agra_OrbitalVCM_Type_c* in, pyramid::domain_model::agra::OrbitalVCM_Type& out);

void to_c(const pyramid::domain_model::agra::VCM_DataType& in, pyramid_data_model_agra_VCM_DataType_c* out);
void from_c(const pyramid_data_model_agra_VCM_DataType_c* in, pyramid::domain_model::agra::VCM_DataType& out);

void to_c(const pyramid::domain_model::agra::RTN_PositionSigmaType& in, pyramid_data_model_agra_RTN_PositionSigmaType_c* out);
void from_c(const pyramid_data_model_agra_RTN_PositionSigmaType_c* in, pyramid::domain_model::agra::RTN_PositionSigmaType& out);

void to_c(const pyramid::domain_model::agra::RTN_VelocitySigmaType& in, pyramid_data_model_agra_RTN_VelocitySigmaType_c* out);
void from_c(const pyramid_data_model_agra_RTN_VelocitySigmaType_c* in, pyramid::domain_model::agra::RTN_VelocitySigmaType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsChoiceType& in, pyramid_data_model_agra_OrbitalKinematicsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsChoiceType_c* in, pyramid::domain_model::agra::OrbitalKinematicsChoiceType& out);

void to_c(const pyramid::domain_model::agra::OrbitalKinematicsObjectRelativeType& in, pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalKinematicsObjectRelativeType_c* in, pyramid::domain_model::agra::OrbitalKinematicsObjectRelativeType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementKinematicConstraintsType& in, pyramid_data_model_agra_MA_RequirementKinematicConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementKinematicConstraintsType_c* in, pyramid::domain_model::agra::MA_RequirementKinematicConstraintsType& out);

void to_c(const pyramid::domain_model::agra::GeoFiltersQueryType& in, pyramid_data_model_agra_GeoFiltersQueryType_c* out);
void from_c(const pyramid_data_model_agra_GeoFiltersQueryType_c* in, pyramid::domain_model::agra::GeoFiltersQueryType& out);

void to_c(const pyramid::domain_model::agra::QueryPET& in, pyramid_data_model_agra_QueryPET_c* out);
void from_c(const pyramid_data_model_agra_QueryPET_c* in, pyramid::domain_model::agra::QueryPET& out);

void to_c(const pyramid::domain_model::agra::ZoneType& in, pyramid_data_model_agra_ZoneType_c* out);
void from_c(const pyramid_data_model_agra_ZoneType_c* in, pyramid::domain_model::agra::ZoneType& out);

void to_c(const pyramid::domain_model::agra::AreaChoiceType& in, pyramid_data_model_agra_AreaChoiceType_c* out);
void from_c(const pyramid_data_model_agra_AreaChoiceType_c* in, pyramid::domain_model::agra::AreaChoiceType& out);

void to_c(const pyramid::domain_model::agra::PolygonType& in, pyramid_data_model_agra_PolygonType_c* out);
void from_c(const pyramid_data_model_agra_PolygonType_c* in, pyramid::domain_model::agra::PolygonType& out);

void to_c(const pyramid::domain_model::agra::PolygonPointChoiceType& in, pyramid_data_model_agra_PolygonPointChoiceType_c* out);
void from_c(const pyramid_data_model_agra_PolygonPointChoiceType_c* in, pyramid::domain_model::agra::PolygonPointChoiceType& out);

void to_c(const pyramid::domain_model::agra::Point2D_Type& in, pyramid_data_model_agra_Point2D_Type_c* out);
void from_c(const pyramid_data_model_agra_Point2D_Type_c* in, pyramid::domain_model::agra::Point2D_Type& out);

void to_c(const pyramid::domain_model::agra::AltitudeRangeType& in, pyramid_data_model_agra_AltitudeRangeType_c* out);
void from_c(const pyramid_data_model_agra_AltitudeRangeType_c* in, pyramid::domain_model::agra::AltitudeRangeType& out);

void to_c(const pyramid::domain_model::agra::PolygonPointChoiceType_Point2D_List& in, pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c* out);
void from_c(const pyramid_data_model_agra_PolygonPointChoiceType_Point2D_List_c* in, pyramid::domain_model::agra::PolygonPointChoiceType_Point2D_List& out);

void to_c(const pyramid::domain_model::agra::PolygonRelativeType& in, pyramid_data_model_agra_PolygonRelativeType_c* out);
void from_c(const pyramid_data_model_agra_PolygonRelativeType_c* in, pyramid::domain_model::agra::PolygonRelativeType& out);

void to_c(const pyramid::domain_model::agra::ReferenceFrameID_Type& in, pyramid_data_model_agra_ReferenceFrameID_Type_c* out);
void from_c(const pyramid_data_model_agra_ReferenceFrameID_Type_c* in, pyramid::domain_model::agra::ReferenceFrameID_Type& out);

void to_c(const pyramid::domain_model::agra::RelativeOffset2D_Type& in, pyramid_data_model_agra_RelativeOffset2D_Type_c* out);
void from_c(const pyramid_data_model_agra_RelativeOffset2D_Type_c* in, pyramid::domain_model::agra::RelativeOffset2D_Type& out);

void to_c(const pyramid::domain_model::agra::Z_ChoiceType& in, pyramid_data_model_agra_Z_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_Z_ChoiceType_c* in, pyramid::domain_model::agra::Z_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::AltitudeOffsetReferenceType& in, pyramid_data_model_agra_AltitudeOffsetReferenceType_c* out);
void from_c(const pyramid_data_model_agra_AltitudeOffsetReferenceType_c* in, pyramid::domain_model::agra::AltitudeOffsetReferenceType& out);

void to_c(const pyramid::domain_model::agra::AltitudeReferenceType& in, pyramid_data_model_agra_AltitudeReferenceType_c* out);
void from_c(const pyramid_data_model_agra_AltitudeReferenceType_c* in, pyramid::domain_model::agra::AltitudeReferenceType& out);

void to_c(const pyramid::domain_model::agra::LocatedEllipseType& in, pyramid_data_model_agra_LocatedEllipseType_c* out);
void from_c(const pyramid_data_model_agra_LocatedEllipseType_c* in, pyramid::domain_model::agra::LocatedEllipseType& out);

void to_c(const pyramid::domain_model::agra::EllipseType& in, pyramid_data_model_agra_EllipseType_c* out);
void from_c(const pyramid_data_model_agra_EllipseType_c* in, pyramid::domain_model::agra::EllipseType& out);

void to_c(const pyramid::domain_model::agra::PointChoiceType& in, pyramid_data_model_agra_PointChoiceType_c* out);
void from_c(const pyramid_data_model_agra_PointChoiceType_c* in, pyramid::domain_model::agra::PointChoiceType& out);

void to_c(const pyramid::domain_model::agra::Point2D_RelativeType& in, pyramid_data_model_agra_Point2D_RelativeType_c* out);
void from_c(const pyramid_data_model_agra_Point2D_RelativeType_c* in, pyramid::domain_model::agra::Point2D_RelativeType& out);

void to_c(const pyramid::domain_model::agra::LocatedRectangleType& in, pyramid_data_model_agra_LocatedRectangleType_c* out);
void from_c(const pyramid_data_model_agra_LocatedRectangleType_c* in, pyramid::domain_model::agra::LocatedRectangleType& out);

void to_c(const pyramid::domain_model::agra::RectangleType& in, pyramid_data_model_agra_RectangleType_c* out);
void from_c(const pyramid_data_model_agra_RectangleType_c* in, pyramid::domain_model::agra::RectangleType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeAreaType& in, pyramid_data_model_agra_SlantRangeAreaType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeAreaType_c* in, pyramid::domain_model::agra::SlantRangeAreaType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeConstraintsType& in, pyramid_data_model_agra_SlantRangeConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeConstraintsType_c* in, pyramid::domain_model::agra::SlantRangeConstraintsType& out);

void to_c(const pyramid::domain_model::agra::AnglePairType& in, pyramid_data_model_agra_AnglePairType_c* out);
void from_c(const pyramid_data_model_agra_AnglePairType_c* in, pyramid::domain_model::agra::AnglePairType& out);

void to_c(const pyramid::domain_model::agra::OrbitalFiltersQueryType& in, pyramid_data_model_agra_OrbitalFiltersQueryType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalFiltersQueryType_c* in, pyramid::domain_model::agra::OrbitalFiltersQueryType& out);

void to_c(const pyramid::domain_model::agra::OpVolumeType& in, pyramid_data_model_agra_OpVolumeType_c* out);
void from_c(const pyramid_data_model_agra_OpVolumeType_c* in, pyramid::domain_model::agra::OpVolumeType& out);

void to_c(const pyramid::domain_model::agra::GeometricVolumeType& in, pyramid_data_model_agra_GeometricVolumeType_c* out);
void from_c(const pyramid_data_model_agra_GeometricVolumeType_c* in, pyramid::domain_model::agra::GeometricVolumeType& out);

void to_c(const pyramid::domain_model::agra::Shape3D_ChoiceType& in, pyramid_data_model_agra_Shape3D_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_Shape3D_ChoiceType_c* in, pyramid::domain_model::agra::Shape3D_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::SphereType& in, pyramid_data_model_agra_SphereType_c* out);
void from_c(const pyramid_data_model_agra_SphereType_c* in, pyramid::domain_model::agra::SphereType& out);

void to_c(const pyramid::domain_model::agra::DomeType& in, pyramid_data_model_agra_DomeType_c* out);
void from_c(const pyramid_data_model_agra_DomeType_c* in, pyramid::domain_model::agra::DomeType& out);

void to_c(const pyramid::domain_model::agra::EllipsoidType& in, pyramid_data_model_agra_EllipsoidType_c* out);
void from_c(const pyramid_data_model_agra_EllipsoidType_c* in, pyramid::domain_model::agra::EllipsoidType& out);

void to_c(const pyramid::domain_model::agra::CylinderType& in, pyramid_data_model_agra_CylinderType_c* out);
void from_c(const pyramid_data_model_agra_CylinderType_c* in, pyramid::domain_model::agra::CylinderType& out);

void to_c(const pyramid::domain_model::agra::ConeType& in, pyramid_data_model_agra_ConeType_c* out);
void from_c(const pyramid_data_model_agra_ConeType_c* in, pyramid::domain_model::agra::ConeType& out);

void to_c(const pyramid::domain_model::agra::RectangularConeType& in, pyramid_data_model_agra_RectangularConeType_c* out);
void from_c(const pyramid_data_model_agra_RectangularConeType_c* in, pyramid::domain_model::agra::RectangularConeType& out);

void to_c(const pyramid::domain_model::agra::ArcVolumeType& in, pyramid_data_model_agra_ArcVolumeType_c* out);
void from_c(const pyramid_data_model_agra_ArcVolumeType_c* in, pyramid::domain_model::agra::ArcVolumeType& out);

void to_c(const pyramid::domain_model::agra::AlongOrbitalArcDeltaType& in, pyramid_data_model_agra_AlongOrbitalArcDeltaType_c* out);
void from_c(const pyramid_data_model_agra_AlongOrbitalArcDeltaType_c* in, pyramid::domain_model::agra::AlongOrbitalArcDeltaType& out);

void to_c(const pyramid::domain_model::agra::IncRaPeriodVolumeType& in, pyramid_data_model_agra_IncRaPeriodVolumeType_c* out);
void from_c(const pyramid_data_model_agra_IncRaPeriodVolumeType_c* in, pyramid::domain_model::agra::IncRaPeriodVolumeType& out);

void to_c(const pyramid::domain_model::agra::KinematicsChoiceType& in, pyramid_data_model_agra_KinematicsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_KinematicsChoiceType_c* in, pyramid::domain_model::agra::KinematicsChoiceType& out);

void to_c(const pyramid::domain_model::agra::OpVolumeKinematicsType& in, pyramid_data_model_agra_OpVolumeKinematicsType_c* out);
void from_c(const pyramid_data_model_agra_OpVolumeKinematicsType_c* in, pyramid::domain_model::agra::OpVolumeKinematicsType& out);

void to_c(const pyramid::domain_model::agra::Velocity2D_Type& in, pyramid_data_model_agra_Velocity2D_Type_c* out);
void from_c(const pyramid_data_model_agra_Velocity2D_Type_c* in, pyramid::domain_model::agra::Velocity2D_Type& out);

void to_c(const pyramid::domain_model::agra::RTN_LocalPositionType& in, pyramid_data_model_agra_RTN_LocalPositionType_c* out);
void from_c(const pyramid_data_model_agra_RTN_LocalPositionType_c* in, pyramid::domain_model::agra::RTN_LocalPositionType& out);

void to_c(const pyramid::domain_model::agra::KinematicsOptionsType& in, pyramid_data_model_agra_KinematicsOptionsType_c* out);
void from_c(const pyramid_data_model_agra_KinematicsOptionsType_c* in, pyramid::domain_model::agra::KinematicsOptionsType& out);

void to_c(const pyramid::domain_model::agra::KinematicsMultiStandardType& in, pyramid_data_model_agra_KinematicsMultiStandardType_c* out);
void from_c(const pyramid_data_model_agra_KinematicsMultiStandardType_c* in, pyramid::domain_model::agra::KinematicsMultiStandardType& out);

void to_c(const pyramid::domain_model::agra::KinematicsType& in, pyramid_data_model_agra_KinematicsType_c* out);
void from_c(const pyramid_data_model_agra_KinematicsType_c* in, pyramid::domain_model::agra::KinematicsType& out);

void to_c(const pyramid::domain_model::agra::EntityPositionType& in, pyramid_data_model_agra_EntityPositionType_c* out);
void from_c(const pyramid_data_model_agra_EntityPositionType_c* in, pyramid::domain_model::agra::EntityPositionType& out);

void to_c(const pyramid::domain_model::agra::FixedPositionType& in, pyramid_data_model_agra_FixedPositionType_c* out);
void from_c(const pyramid_data_model_agra_FixedPositionType_c* in, pyramid::domain_model::agra::FixedPositionType& out);

void to_c(const pyramid::domain_model::agra::Point2D_ReportedType& in, pyramid_data_model_agra_Point2D_ReportedType_c* out);
void from_c(const pyramid_data_model_agra_Point2D_ReportedType_c* in, pyramid::domain_model::agra::Point2D_ReportedType& out);

void to_c(const pyramid::domain_model::agra::UncertaintyType& in, pyramid_data_model_agra_UncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_UncertaintyType_c* in, pyramid::domain_model::agra::UncertaintyType& out);

void to_c(const pyramid::domain_model::agra::RelativePositionType& in, pyramid_data_model_agra_RelativePositionType_c* out);
void from_c(const pyramid_data_model_agra_RelativePositionType_c* in, pyramid::domain_model::agra::RelativePositionType& out);

void to_c(const pyramid::domain_model::agra::Point2D_ReferenceType& in, pyramid_data_model_agra_Point2D_ReferenceType_c* out);
void from_c(const pyramid_data_model_agra_Point2D_ReferenceType_c* in, pyramid::domain_model::agra::Point2D_ReferenceType& out);

void to_c(const pyramid::domain_model::agra::ReferenceObjectType& in, pyramid_data_model_agra_ReferenceObjectType_c* out);
void from_c(const pyramid_data_model_agra_ReferenceObjectType_c* in, pyramid::domain_model::agra::ReferenceObjectType& out);

void to_c(const pyramid::domain_model::agra::OpPointID_Type& in, pyramid_data_model_agra_OpPointID_Type_c* out);
void from_c(const pyramid_data_model_agra_OpPointID_Type_c* in, pyramid::domain_model::agra::OpPointID_Type& out);

void to_c(const pyramid::domain_model::agra::LineOfSightChoiceType& in, pyramid_data_model_agra_LineOfSightChoiceType_c* out);
void from_c(const pyramid_data_model_agra_LineOfSightChoiceType_c* in, pyramid::domain_model::agra::LineOfSightChoiceType& out);

void to_c(const pyramid::domain_model::agra::LOS_MeasurementAndUncertaintyType& in, pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_LOS_MeasurementAndUncertaintyType_c* in, pyramid::domain_model::agra::LOS_MeasurementAndUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::LOS_MeasurementType& in, pyramid_data_model_agra_LOS_MeasurementType_c* out);
void from_c(const pyramid_data_model_agra_LOS_MeasurementType_c* in, pyramid::domain_model::agra::LOS_MeasurementType& out);

void to_c(const pyramid::domain_model::agra::LOS_AzElType& in, pyramid_data_model_agra_LOS_AzElType_c* out);
void from_c(const pyramid_data_model_agra_LOS_AzElType_c* in, pyramid::domain_model::agra::LOS_AzElType& out);

void to_c(const pyramid::domain_model::agra::LOS_AzElRatesType& in, pyramid_data_model_agra_LOS_AzElRatesType_c* out);
void from_c(const pyramid_data_model_agra_LOS_AzElRatesType_c* in, pyramid::domain_model::agra::LOS_AzElRatesType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleType& in, pyramid_data_model_agra_ConeAngleType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleType_c* in, pyramid::domain_model::agra::ConeAngleType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleRatesType& in, pyramid_data_model_agra_ConeAngleRatesType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleRatesType_c* in, pyramid::domain_model::agra::ConeAngleRatesType& out);

void to_c(const pyramid::domain_model::agra::ArrivalDataType& in, pyramid_data_model_agra_ArrivalDataType_c* out);
void from_c(const pyramid_data_model_agra_ArrivalDataType_c* in, pyramid::domain_model::agra::ArrivalDataType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeType& in, pyramid_data_model_agra_SlantRangeType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeType_c* in, pyramid::domain_model::agra::SlantRangeType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeRatesAndAccelerationType& in, pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeRatesAndAccelerationType_c* in, pyramid::domain_model::agra::SlantRangeRatesAndAccelerationType& out);

void to_c(const pyramid::domain_model::agra::LOS_MeasurementUncertaintyErrorSourcesType& in, pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c* out);
void from_c(const pyramid_data_model_agra_LOS_MeasurementUncertaintyErrorSourcesType_c* in, pyramid::domain_model::agra::LOS_MeasurementUncertaintyErrorSourcesType& out);

void to_c(const pyramid::domain_model::agra::LOS_MeasurementUncertaintyType& in, pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_LOS_MeasurementUncertaintyType_c* in, pyramid::domain_model::agra::LOS_MeasurementUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::LOS_VarianceAndCovarianceType& in, pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_LOS_VarianceAndCovarianceType_c* in, pyramid::domain_model::agra::LOS_VarianceAndCovarianceType& out);

void to_c(const pyramid::domain_model::agra::LOS_VarianceType& in, pyramid_data_model_agra_LOS_VarianceType_c* out);
void from_c(const pyramid_data_model_agra_LOS_VarianceType_c* in, pyramid::domain_model::agra::LOS_VarianceType& out);

void to_c(const pyramid::domain_model::agra::LOS_VarianceRatesType& in, pyramid_data_model_agra_LOS_VarianceRatesType_c* out);
void from_c(const pyramid_data_model_agra_LOS_VarianceRatesType_c* in, pyramid::domain_model::agra::LOS_VarianceRatesType& out);

void to_c(const pyramid::domain_model::agra::LOS_CovarianceType& in, pyramid_data_model_agra_LOS_CovarianceType_c* out);
void from_c(const pyramid_data_model_agra_LOS_CovarianceType_c* in, pyramid::domain_model::agra::LOS_CovarianceType& out);

void to_c(const pyramid::domain_model::agra::LOS_CovariancesRatesType& in, pyramid_data_model_agra_LOS_CovariancesRatesType_c* out);
void from_c(const pyramid_data_model_agra_LOS_CovariancesRatesType_c* in, pyramid::domain_model::agra::LOS_CovariancesRatesType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleUncertaintyType& in, pyramid_data_model_agra_ConeAngleUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleUncertaintyType_c* in, pyramid::domain_model::agra::ConeAngleUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleVarianceType& in, pyramid_data_model_agra_ConeAngleVarianceType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleVarianceType_c* in, pyramid::domain_model::agra::ConeAngleVarianceType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleVarianceRatesType& in, pyramid_data_model_agra_ConeAngleVarianceRatesType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleVarianceRatesType_c* in, pyramid::domain_model::agra::ConeAngleVarianceRatesType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleCovarianceType& in, pyramid_data_model_agra_ConeAngleCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleCovarianceType_c* in, pyramid::domain_model::agra::ConeAngleCovarianceType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleCovarianceRatesType& in, pyramid_data_model_agra_ConeAngleCovarianceRatesType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleCovarianceRatesType_c* in, pyramid::domain_model::agra::ConeAngleCovarianceRatesType& out);

void to_c(const pyramid::domain_model::agra::ArrivalDataUncertaintyType& in, pyramid_data_model_agra_ArrivalDataUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_ArrivalDataUncertaintyType_c* in, pyramid::domain_model::agra::ArrivalDataUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::ArrivalDataVarianceType& in, pyramid_data_model_agra_ArrivalDataVarianceType_c* out);
void from_c(const pyramid_data_model_agra_ArrivalDataVarianceType_c* in, pyramid::domain_model::agra::ArrivalDataVarianceType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeUncertaintyType& in, pyramid_data_model_agra_SlantRangeUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeUncertaintyType_c* in, pyramid::domain_model::agra::SlantRangeUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeVarianceType& in, pyramid_data_model_agra_SlantRangeVarianceType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeVarianceType_c* in, pyramid::domain_model::agra::SlantRangeVarianceType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeVarianceRatesAndAccelerationType& in, pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeVarianceRatesAndAccelerationType_c* in, pyramid::domain_model::agra::SlantRangeVarianceRatesAndAccelerationType& out);

void to_c(const pyramid::domain_model::agra::SlantRangeCovarianceType& in, pyramid_data_model_agra_SlantRangeCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_SlantRangeCovarianceType_c* in, pyramid::domain_model::agra::SlantRangeCovarianceType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleSlantRangeUncertaintyType& in, pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleSlantRangeUncertaintyType_c* in, pyramid::domain_model::agra::ConeAngleSlantRangeUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleSlantRangeCovarianceType& in, pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleSlantRangeCovarianceType_c* in, pyramid::domain_model::agra::ConeAngleSlantRangeCovarianceType& out);

void to_c(const pyramid::domain_model::agra::ConeAngleSlantRangeCovarianceRatesType& in, pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c* out);
void from_c(const pyramid_data_model_agra_ConeAngleSlantRangeCovarianceRatesType_c* in, pyramid::domain_model::agra::ConeAngleSlantRangeCovarianceRatesType& out);

void to_c(const pyramid::domain_model::agra::LOS_SlantRangeUncertaintyType& in, pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_LOS_SlantRangeUncertaintyType_c* in, pyramid::domain_model::agra::LOS_SlantRangeUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::LOS_SlantRangeCovarianceType& in, pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_LOS_SlantRangeCovarianceType_c* in, pyramid::domain_model::agra::LOS_SlantRangeCovarianceType& out);

void to_c(const pyramid::domain_model::agra::LOS_SlantRangeCovarianceRatesType& in, pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c* out);
void from_c(const pyramid_data_model_agra_LOS_SlantRangeCovarianceRatesType_c* in, pyramid::domain_model::agra::LOS_SlantRangeCovarianceRatesType& out);

void to_c(const pyramid::domain_model::agra::LOS_UncertaintyType& in, pyramid_data_model_agra_LOS_UncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_LOS_UncertaintyType_c* in, pyramid::domain_model::agra::LOS_UncertaintyType& out);

void to_c(const pyramid::domain_model::agra::LOS3D_KinematicsType& in, pyramid_data_model_agra_LOS3D_KinematicsType_c* out);
void from_c(const pyramid_data_model_agra_LOS3D_KinematicsType_c* in, pyramid::domain_model::agra::LOS3D_KinematicsType& out);

void to_c(const pyramid::domain_model::agra::RelativeAnglesLOS3D_Type& in, pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c* out);
void from_c(const pyramid_data_model_agra_RelativeAnglesLOS3D_Type_c* in, pyramid::domain_model::agra::RelativeAnglesLOS3D_Type& out);

void to_c(const pyramid::domain_model::agra::UnitVectorType& in, pyramid_data_model_agra_UnitVectorType_c* out);
void from_c(const pyramid_data_model_agra_UnitVectorType_c* in, pyramid::domain_model::agra::UnitVectorType& out);

void to_c(const pyramid::domain_model::agra::RelativeAngleUncertaintyLOS3D_Type& in, pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c* out);
void from_c(const pyramid_data_model_agra_RelativeAngleUncertaintyLOS3D_Type_c* in, pyramid::domain_model::agra::RelativeAngleUncertaintyLOS3D_Type& out);

void to_c(const pyramid::domain_model::agra::RelativeAngleRateUncertaintyLOS3D_Type& in, pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c* out);
void from_c(const pyramid_data_model_agra_RelativeAngleRateUncertaintyLOS3D_Type_c* in, pyramid::domain_model::agra::RelativeAngleRateUncertaintyLOS3D_Type& out);

void to_c(const pyramid::domain_model::agra::RelativeSlantRangeLOS3D_Type& in, pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c* out);
void from_c(const pyramid_data_model_agra_RelativeSlantRangeLOS3D_Type_c* in, pyramid::domain_model::agra::RelativeSlantRangeLOS3D_Type& out);

void to_c(const pyramid::domain_model::agra::LOS3D_CovarianceType& in, pyramid_data_model_agra_LOS3D_CovarianceType_c* out);
void from_c(const pyramid_data_model_agra_LOS3D_CovarianceType_c* in, pyramid::domain_model::agra::LOS3D_CovarianceType& out);

void to_c(const pyramid::domain_model::agra::Velocity2D_UncertaintyType& in, pyramid_data_model_agra_Velocity2D_UncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_Velocity2D_UncertaintyType_c* in, pyramid::domain_model::agra::Velocity2D_UncertaintyType& out);

void to_c(const pyramid::domain_model::agra::Acceleration3D_Type& in, pyramid_data_model_agra_Acceleration3D_Type_c* out);
void from_c(const pyramid_data_model_agra_Acceleration3D_Type_c* in, pyramid::domain_model::agra::Acceleration3D_Type& out);

void to_c(const pyramid::domain_model::agra::StateCovarianceNED_Type& in, pyramid_data_model_agra_StateCovarianceNED_Type_c* out);
void from_c(const pyramid_data_model_agra_StateCovarianceNED_Type_c* in, pyramid::domain_model::agra::StateCovarianceNED_Type& out);

void to_c(const pyramid::domain_model::agra::PositionAndVelocityCovarianceType& in, pyramid_data_model_agra_PositionAndVelocityCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_PositionAndVelocityCovarianceType_c* in, pyramid::domain_model::agra::PositionAndVelocityCovarianceType& out);

void to_c(const pyramid::domain_model::agra::PositionPositionCovarianceType& in, pyramid_data_model_agra_PositionPositionCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_PositionPositionCovarianceType_c* in, pyramid::domain_model::agra::PositionPositionCovarianceType& out);

void to_c(const pyramid::domain_model::agra::PositionVelocityCovarianceType& in, pyramid_data_model_agra_PositionVelocityCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_PositionVelocityCovarianceType_c* in, pyramid::domain_model::agra::PositionVelocityCovarianceType& out);

void to_c(const pyramid::domain_model::agra::VelocityVelocityCovarianceType& in, pyramid_data_model_agra_VelocityVelocityCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_VelocityVelocityCovarianceType_c* in, pyramid::domain_model::agra::VelocityVelocityCovarianceType& out);

void to_c(const pyramid::domain_model::agra::AccelerationAccelerationCovarianceType& in, pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_AccelerationAccelerationCovarianceType_c* in, pyramid::domain_model::agra::AccelerationAccelerationCovarianceType& out);

void to_c(const pyramid::domain_model::agra::PositionAccelerationCovarianceType& in, pyramid_data_model_agra_PositionAccelerationCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_PositionAccelerationCovarianceType_c* in, pyramid::domain_model::agra::PositionAccelerationCovarianceType& out);

void to_c(const pyramid::domain_model::agra::VelocityAccelerationCovarianceType& in, pyramid_data_model_agra_VelocityAccelerationCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_VelocityAccelerationCovarianceType_c* in, pyramid::domain_model::agra::VelocityAccelerationCovarianceType& out);

void to_c(const pyramid::domain_model::agra::OrientationType& in, pyramid_data_model_agra_OrientationType_c* out);
void from_c(const pyramid_data_model_agra_OrientationType_c* in, pyramid::domain_model::agra::OrientationType& out);

void to_c(const pyramid::domain_model::agra::OrientationCovarianceType& in, pyramid_data_model_agra_OrientationCovarianceType_c* out);
void from_c(const pyramid_data_model_agra_OrientationCovarianceType_c* in, pyramid::domain_model::agra::OrientationCovarianceType& out);

void to_c(const pyramid::domain_model::agra::AngleHalfPairType& in, pyramid_data_model_agra_AngleHalfPairType_c* out);
void from_c(const pyramid_data_model_agra_AngleHalfPairType_c* in, pyramid::domain_model::agra::AngleHalfPairType& out);

void to_c(const pyramid::domain_model::agra::GeocentricVolumeType& in, pyramid_data_model_agra_GeocentricVolumeType_c* out);
void from_c(const pyramid_data_model_agra_GeocentricVolumeType_c* in, pyramid::domain_model::agra::GeocentricVolumeType& out);

void to_c(const pyramid::domain_model::agra::RequirementDependencyType& in, pyramid_data_model_agra_RequirementDependencyType_c* out);
void from_c(const pyramid_data_model_agra_RequirementDependencyType_c* in, pyramid::domain_model::agra::RequirementDependencyType& out);

void to_c(const pyramid::domain_model::agra::RequirementDependencyBaseType& in, pyramid_data_model_agra_RequirementDependencyBaseType_c* out);
void from_c(const pyramid_data_model_agra_RequirementDependencyBaseType_c* in, pyramid::domain_model::agra::RequirementDependencyBaseType& out);

void to_c(const pyramid::domain_model::agra::RequirementInstanceID_ChoiceType& in, pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_RequirementInstanceID_ChoiceType_c* in, pyramid::domain_model::agra::RequirementInstanceID_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::EffectID_Type& in, pyramid_data_model_agra_EffectID_Type_c* out);
void from_c(const pyramid_data_model_agra_EffectID_Type_c* in, pyramid::domain_model::agra::EffectID_Type& out);

void to_c(const pyramid::domain_model::agra::TaskID_Type& in, pyramid_data_model_agra_TaskID_Type_c* out);
void from_c(const pyramid_data_model_agra_TaskID_Type_c* in, pyramid::domain_model::agra::TaskID_Type& out);

void to_c(const pyramid::domain_model::agra::CommandID_Type& in, pyramid_data_model_agra_CommandID_Type_c* out);
void from_c(const pyramid_data_model_agra_CommandID_Type_c* in, pyramid::domain_model::agra::CommandID_Type& out);

void to_c(const pyramid::domain_model::agra::ResponseID_Type& in, pyramid_data_model_agra_ResponseID_Type_c* out);
void from_c(const pyramid_data_model_agra_ResponseID_Type_c* in, pyramid::domain_model::agra::ResponseID_Type& out);

void to_c(const pyramid::domain_model::agra::DataProductClassificationLevelType& in, pyramid_data_model_agra_DataProductClassificationLevelType_c* out);
void from_c(const pyramid_data_model_agra_DataProductClassificationLevelType_c* in, pyramid::domain_model::agra::DataProductClassificationLevelType& out);

void to_c(const pyramid::domain_model::agra::MA_AnalyticConstraintsType& in, pyramid_data_model_agra_MA_AnalyticConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_MA_AnalyticConstraintsType_c* in, pyramid::domain_model::agra::MA_AnalyticConstraintsType& out);

void to_c(const pyramid::domain_model::agra::OpConstraintWeightingType& in, pyramid_data_model_agra_OpConstraintWeightingType_c* out);
void from_c(const pyramid_data_model_agra_OpConstraintWeightingType_c* in, pyramid::domain_model::agra::OpConstraintWeightingType& out);

void to_c(const pyramid::domain_model::agra::OpConstraintWeightingValueType& in, pyramid_data_model_agra_OpConstraintWeightingValueType_c* out);
void from_c(const pyramid_data_model_agra_OpConstraintWeightingValueType_c* in, pyramid::domain_model::agra::OpConstraintWeightingValueType& out);

void to_c(const pyramid::domain_model::agra::PercentRangeType& in, pyramid_data_model_agra_PercentRangeType_c* out);
void from_c(const pyramid_data_model_agra_PercentRangeType_c* in, pyramid::domain_model::agra::PercentRangeType& out);

void to_c(const pyramid::domain_model::agra::FileLocationID_Type& in, pyramid_data_model_agra_FileLocationID_Type_c* out);
void from_c(const pyramid_data_model_agra_FileLocationID_Type_c* in, pyramid::domain_model::agra::FileLocationID_Type& out);

void to_c(const pyramid::domain_model::agra::SurvivabilityRiskSettingType& in, pyramid_data_model_agra_SurvivabilityRiskSettingType_c* out);
void from_c(const pyramid_data_model_agra_SurvivabilityRiskSettingType_c* in, pyramid::domain_model::agra::SurvivabilityRiskSettingType& out);

void to_c(const pyramid::domain_model::agra::RiskSettingType& in, pyramid_data_model_agra_RiskSettingType_c* out);
void from_c(const pyramid_data_model_agra_RiskSettingType_c* in, pyramid::domain_model::agra::RiskSettingType& out);

void to_c(const pyramid::domain_model::agra::VulnerabilityLevelsType& in, pyramid_data_model_agra_VulnerabilityLevelsType_c* out);
void from_c(const pyramid_data_model_agra_VulnerabilityLevelsType_c* in, pyramid::domain_model::agra::VulnerabilityLevelsType& out);

void to_c(const pyramid::domain_model::agra::MA_AccessAssessmentFilterType& in, pyramid_data_model_agra_MA_AccessAssessmentFilterType_c* out);
void from_c(const pyramid_data_model_agra_MA_AccessAssessmentFilterType_c* in, pyramid::domain_model::agra::MA_AccessAssessmentFilterType& out);

void to_c(const pyramid::domain_model::agra::CapabilityTaxonomyUniversalType& in, pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c* out);
void from_c(const pyramid_data_model_agra_CapabilityTaxonomyUniversalType_c* in, pyramid::domain_model::agra::CapabilityTaxonomyUniversalType& out);

void to_c(const pyramid::domain_model::agra::CapabilityTaxonomyUniversalBaseType& in, pyramid_data_model_agra_CapabilityTaxonomyUniversalBaseType_c* out);
void from_c(const pyramid_data_model_agra_CapabilityTaxonomyUniversalBaseType_c* in, pyramid::domain_model::agra::CapabilityTaxonomyUniversalBaseType& out);

void to_c(const pyramid::domain_model::agra::CapabilityTaxonomyType& in, pyramid_data_model_agra_CapabilityTaxonomyType_c* out);
void from_c(const pyramid_data_model_agra_CapabilityTaxonomyType_c* in, pyramid::domain_model::agra::CapabilityTaxonomyType& out);

void to_c(const pyramid::domain_model::agra::AMTI_SpecificDataType& in, pyramid_data_model_agra_AMTI_SpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_AMTI_SpecificDataType_c* in, pyramid::domain_model::agra::AMTI_SpecificDataType& out);

void to_c(const pyramid::domain_model::agra::CargoDeliverySpecificDataType& in, pyramid_data_model_agra_CargoDeliverySpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_CargoDeliverySpecificDataType_c* in, pyramid::domain_model::agra::CargoDeliverySpecificDataType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SpecificDataType& in, pyramid_data_model_agra_COMINT_SpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SpecificDataType_c* in, pyramid::domain_model::agra::COMINT_SpecificDataType& out);

void to_c(const pyramid::domain_model::agra::ESM_SpecificDataType& in, pyramid_data_model_agra_ESM_SpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_ESM_SpecificDataType_c* in, pyramid::domain_model::agra::ESM_SpecificDataType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSpecificDataType& in, pyramid_data_model_agra_OrbitalSurveillanceSpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSpecificDataType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSpecificDataType& out);

void to_c(const pyramid::domain_model::agra::SAR_SpecificDataType& in, pyramid_data_model_agra_SAR_SpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_SAR_SpecificDataType_c* in, pyramid::domain_model::agra::SAR_SpecificDataType& out);

void to_c(const pyramid::domain_model::agra::SMTI_SpecificDataType& in, pyramid_data_model_agra_SMTI_SpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_SpecificDataType_c* in, pyramid::domain_model::agra::SMTI_SpecificDataType& out);

void to_c(const pyramid::domain_model::agra::MA_AssetFilterType& in, pyramid_data_model_agra_MA_AssetFilterType_c* out);
void from_c(const pyramid_data_model_agra_MA_AssetFilterType_c* in, pyramid::domain_model::agra::MA_AssetFilterType& out);

void to_c(const pyramid::domain_model::agra::MA_SystemFilterType& in, pyramid_data_model_agra_MA_SystemFilterType_c* out);
void from_c(const pyramid_data_model_agra_MA_SystemFilterType_c* in, pyramid::domain_model::agra::MA_SystemFilterType& out);

void to_c(const pyramid::domain_model::agra::MA_SystemComparativeType& in, pyramid_data_model_agra_MA_SystemComparativeType_c* out);
void from_c(const pyramid_data_model_agra_MA_SystemComparativeType_c* in, pyramid::domain_model::agra::MA_SystemComparativeType& out);

void to_c(const pyramid::domain_model::agra::MA_SystemCharacteristicType& in, pyramid_data_model_agra_MA_SystemCharacteristicType_c* out);
void from_c(const pyramid_data_model_agra_MA_SystemCharacteristicType_c* in, pyramid::domain_model::agra::MA_SystemCharacteristicType& out);

void to_c(const pyramid::domain_model::agra::EntityIdentityChoiceType& in, pyramid_data_model_agra_EntityIdentityChoiceType_c* out);
void from_c(const pyramid_data_model_agra_EntityIdentityChoiceType_c* in, pyramid::domain_model::agra::EntityIdentityChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_PrioritizationListValueType& in, pyramid_data_model_agra_MA_PrioritizationListValueType_c* out);
void from_c(const pyramid_data_model_agra_MA_PrioritizationListValueType_c* in, pyramid::domain_model::agra::MA_PrioritizationListValueType& out);

void to_c(const pyramid::domain_model::agra::MA_PrioritizationType& in, pyramid_data_model_agra_MA_PrioritizationType_c* out);
void from_c(const pyramid_data_model_agra_MA_PrioritizationType_c* in, pyramid::domain_model::agra::MA_PrioritizationType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementTaxonomyType& in, pyramid_data_model_agra_MA_RequirementTaxonomyType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementTaxonomyType_c* in, pyramid::domain_model::agra::MA_RequirementTaxonomyType& out);

void to_c(const pyramid::domain_model::agra::BehaviorType& in, pyramid_data_model_agra_BehaviorType_c* out);
void from_c(const pyramid_data_model_agra_BehaviorType_c* in, pyramid::domain_model::agra::BehaviorType& out);

void to_c(const pyramid::domain_model::agra::ActivityByType& in, pyramid_data_model_agra_ActivityByType_c* out);
void from_c(const pyramid_data_model_agra_ActivityByType_c* in, pyramid::domain_model::agra::ActivityByType& out);

void to_c(const pyramid::domain_model::agra::MA_EntityFilterType& in, pyramid_data_model_agra_MA_EntityFilterType_c* out);
void from_c(const pyramid_data_model_agra_MA_EntityFilterType_c* in, pyramid::domain_model::agra::MA_EntityFilterType& out);

void to_c(const pyramid::domain_model::agra::MA_DesignationFilterType& in, pyramid_data_model_agra_MA_DesignationFilterType_c* out);
void from_c(const pyramid_data_model_agra_MA_DesignationFilterType_c* in, pyramid::domain_model::agra::MA_DesignationFilterType& out);

void to_c(const pyramid::domain_model::agra::MA_EntityComparativeType& in, pyramid_data_model_agra_MA_EntityComparativeType_c* out);
void from_c(const pyramid_data_model_agra_MA_EntityComparativeType_c* in, pyramid::domain_model::agra::MA_EntityComparativeType& out);

void to_c(const pyramid::domain_model::agra::MA_EntityCharacteristicType& in, pyramid_data_model_agra_MA_EntityCharacteristicType_c* out);
void from_c(const pyramid_data_model_agra_MA_EntityCharacteristicType_c* in, pyramid::domain_model::agra::MA_EntityCharacteristicType& out);

void to_c(const pyramid::domain_model::agra::IdentityComparisonType& in, pyramid_data_model_agra_IdentityComparisonType_c* out);
void from_c(const pyramid_data_model_agra_IdentityComparisonType_c* in, pyramid::domain_model::agra::IdentityComparisonType& out);

void to_c(const pyramid::domain_model::agra::AccessEventFilterType& in, pyramid_data_model_agra_AccessEventFilterType_c* out);
void from_c(const pyramid_data_model_agra_AccessEventFilterType_c* in, pyramid::domain_model::agra::AccessEventFilterType& out);

void to_c(const pyramid::domain_model::agra::DurationRangeType& in, pyramid_data_model_agra_DurationRangeType_c* out);
void from_c(const pyramid_data_model_agra_DurationRangeType_c* in, pyramid::domain_model::agra::DurationRangeType& out);

void to_c(const pyramid::domain_model::agra::DistanceConstraintsType& in, pyramid_data_model_agra_DistanceConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_DistanceConstraintsType_c* in, pyramid::domain_model::agra::DistanceConstraintsType& out);

void to_c(const pyramid::domain_model::agra::RF_TaskPerformanceType& in, pyramid_data_model_agra_RF_TaskPerformanceType_c* out);
void from_c(const pyramid_data_model_agra_RF_TaskPerformanceType_c* in, pyramid::domain_model::agra::RF_TaskPerformanceType& out);

void to_c(const pyramid::domain_model::agra::RF_TaskPerformanceConstraintType& in, pyramid_data_model_agra_RF_TaskPerformanceConstraintType_c* out);
void from_c(const pyramid_data_model_agra_RF_TaskPerformanceConstraintType_c* in, pyramid::domain_model::agra::RF_TaskPerformanceConstraintType& out);

void to_c(const pyramid::domain_model::agra::PerformanceMetricSetType& in, pyramid_data_model_agra_PerformanceMetricSetType_c* out);
void from_c(const pyramid_data_model_agra_PerformanceMetricSetType_c* in, pyramid::domain_model::agra::PerformanceMetricSetType& out);

void to_c(const pyramid::domain_model::agra::MetricValueType& in, pyramid_data_model_agra_MetricValueType_c* out);
void from_c(const pyramid_data_model_agra_MetricValueType_c* in, pyramid::domain_model::agra::MetricValueType& out);

void to_c(const pyramid::domain_model::agra::RF_TaskNormalizedMetricsType& in, pyramid_data_model_agra_RF_TaskNormalizedMetricsType_c* out);
void from_c(const pyramid_data_model_agra_RF_TaskNormalizedMetricsType_c* in, pyramid::domain_model::agra::RF_TaskNormalizedMetricsType& out);

void to_c(const pyramid::domain_model::agra::NormalizationTableType& in, pyramid_data_model_agra_NormalizationTableType_c* out);
void from_c(const pyramid_data_model_agra_NormalizationTableType_c* in, pyramid::domain_model::agra::NormalizationTableType& out);

void to_c(const pyramid::domain_model::agra::MA_ShotDoctrineParametersType& in, pyramid_data_model_agra_MA_ShotDoctrineParametersType_c* out);
void from_c(const pyramid_data_model_agra_MA_ShotDoctrineParametersType_c* in, pyramid::domain_model::agra::MA_ShotDoctrineParametersType& out);

void to_c(const pyramid::domain_model::agra::RequirementGuidanceType& in, pyramid_data_model_agra_RequirementGuidanceType_c* out);
void from_c(const pyramid_data_model_agra_RequirementGuidanceType_c* in, pyramid::domain_model::agra::RequirementGuidanceType& out);

void to_c(const pyramid::domain_model::agra::MA_UniqueActionDataType& in, pyramid_data_model_agra_MA_UniqueActionDataType_c* out);
void from_c(const pyramid_data_model_agra_MA_UniqueActionDataType_c* in, pyramid::domain_model::agra::MA_UniqueActionDataType& out);

void to_c(const pyramid::domain_model::agra::MA_AttackKineticParametersType& in, pyramid_data_model_agra_MA_AttackKineticParametersType_c* out);
void from_c(const pyramid_data_model_agra_MA_AttackKineticParametersType_c* in, pyramid::domain_model::agra::MA_AttackKineticParametersType& out);

void to_c(const pyramid::domain_model::agra::MA_EngagementParametersType& in, pyramid_data_model_agra_MA_EngagementParametersType_c* out);
void from_c(const pyramid_data_model_agra_MA_EngagementParametersType_c* in, pyramid::domain_model::agra::MA_EngagementParametersType& out);

void to_c(const pyramid::domain_model::agra::MA_InterceptTacticType& in, pyramid_data_model_agra_MA_InterceptTacticType_c* out);
void from_c(const pyramid_data_model_agra_MA_InterceptTacticType_c* in, pyramid::domain_model::agra::MA_InterceptTacticType& out);

void to_c(const pyramid::domain_model::agra::MA_SkateType& in, pyramid_data_model_agra_MA_SkateType_c* out);
void from_c(const pyramid_data_model_agra_MA_SkateType_c* in, pyramid::domain_model::agra::MA_SkateType& out);

void to_c(const pyramid::domain_model::agra::MA_BanzaiType& in, pyramid_data_model_agra_MA_BanzaiType_c* out);
void from_c(const pyramid_data_model_agra_MA_BanzaiType_c* in, pyramid::domain_model::agra::MA_BanzaiType& out);

void to_c(const pyramid::domain_model::agra::IdentityKindInstanceType& in, pyramid_data_model_agra_IdentityKindInstanceType_c* out);
void from_c(const pyramid_data_model_agra_IdentityKindInstanceType_c* in, pyramid::domain_model::agra::IdentityKindInstanceType& out);

void to_c(const pyramid::domain_model::agra::TargetType& in, pyramid_data_model_agra_TargetType_c* out);
void from_c(const pyramid_data_model_agra_TargetType_c* in, pyramid::domain_model::agra::TargetType& out);

void to_c(const pyramid::domain_model::agra::OperatorLocationOfInterestID_Type& in, pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c* out);
void from_c(const pyramid_data_model_agra_OperatorLocationOfInterestID_Type_c* in, pyramid::domain_model::agra::OperatorLocationOfInterestID_Type& out);

void to_c(const pyramid::domain_model::agra::SignalID_Type& in, pyramid_data_model_agra_SignalID_Type_c* out);
void from_c(const pyramid_data_model_agra_SignalID_Type_c* in, pyramid::domain_model::agra::SignalID_Type& out);

void to_c(const pyramid::domain_model::agra::OpZoneID_Type& in, pyramid_data_model_agra_OpZoneID_Type_c* out);
void from_c(const pyramid_data_model_agra_OpZoneID_Type_c* in, pyramid::domain_model::agra::OpZoneID_Type& out);

void to_c(const pyramid::domain_model::agra::OpVolumeID_Type& in, pyramid_data_model_agra_OpVolumeID_Type_c* out);
void from_c(const pyramid_data_model_agra_OpVolumeID_Type_c* in, pyramid::domain_model::agra::OpVolumeID_Type& out);

void to_c(const pyramid::domain_model::agra::OpLineID_Type& in, pyramid_data_model_agra_OpLineID_Type_c* out);
void from_c(const pyramid_data_model_agra_OpLineID_Type_c* in, pyramid::domain_model::agra::OpLineID_Type& out);

void to_c(const pyramid::domain_model::agra::PointTargetType& in, pyramid_data_model_agra_PointTargetType_c* out);
void from_c(const pyramid_data_model_agra_PointTargetType_c* in, pyramid::domain_model::agra::PointTargetType& out);

void to_c(const pyramid::domain_model::agra::ZoneExternalType& in, pyramid_data_model_agra_ZoneExternalType_c* out);
void from_c(const pyramid_data_model_agra_ZoneExternalType_c* in, pyramid::domain_model::agra::ZoneExternalType& out);

void to_c(const pyramid::domain_model::agra::LineTargetType& in, pyramid_data_model_agra_LineTargetType_c* out);
void from_c(const pyramid_data_model_agra_LineTargetType_c* in, pyramid::domain_model::agra::LineTargetType& out);

void to_c(const pyramid::domain_model::agra::LineType& in, pyramid_data_model_agra_LineType_c* out);
void from_c(const pyramid_data_model_agra_LineType_c* in, pyramid::domain_model::agra::LineType& out);

void to_c(const pyramid::domain_model::agra::LinePointChoiceType& in, pyramid_data_model_agra_LinePointChoiceType_c* out);
void from_c(const pyramid_data_model_agra_LinePointChoiceType_c* in, pyramid::domain_model::agra::LinePointChoiceType& out);

void to_c(const pyramid::domain_model::agra::LinePoint2D_Type& in, pyramid_data_model_agra_LinePoint2D_Type_c* out);
void from_c(const pyramid_data_model_agra_LinePoint2D_Type_c* in, pyramid::domain_model::agra::LinePoint2D_Type& out);

void to_c(const pyramid::domain_model::agra::LinePointChoiceType_Point_List& in, pyramid_data_model_agra_LinePointChoiceType_Point_List_c* out);
void from_c(const pyramid_data_model_agra_LinePointChoiceType_Point_List_c* in, pyramid::domain_model::agra::LinePointChoiceType_Point_List& out);

void to_c(const pyramid::domain_model::agra::LineRelativeType& in, pyramid_data_model_agra_LineRelativeType_c* out);
void from_c(const pyramid_data_model_agra_LineRelativeType_c* in, pyramid::domain_model::agra::LineRelativeType& out);

void to_c(const pyramid::domain_model::agra::RequirementTargetConstraintsType& in, pyramid_data_model_agra_RequirementTargetConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTargetConstraintsType_c* in, pyramid::domain_model::agra::RequirementTargetConstraintsType& out);

void to_c(const pyramid::domain_model::agra::AnalyticConstraintsType& in, pyramid_data_model_agra_AnalyticConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_AnalyticConstraintsType_c* in, pyramid::domain_model::agra::AnalyticConstraintsType& out);

void to_c(const pyramid::domain_model::agra::AccessAssessmentFilterType& in, pyramid_data_model_agra_AccessAssessmentFilterType_c* out);
void from_c(const pyramid_data_model_agra_AccessAssessmentFilterType_c* in, pyramid::domain_model::agra::AccessAssessmentFilterType& out);

void to_c(const pyramid::domain_model::agra::AssetFilterType& in, pyramid_data_model_agra_AssetFilterType_c* out);
void from_c(const pyramid_data_model_agra_AssetFilterType_c* in, pyramid::domain_model::agra::AssetFilterType& out);

void to_c(const pyramid::domain_model::agra::SystemFilterType& in, pyramid_data_model_agra_SystemFilterType_c* out);
void from_c(const pyramid_data_model_agra_SystemFilterType_c* in, pyramid::domain_model::agra::SystemFilterType& out);

void to_c(const pyramid::domain_model::agra::SystemComparativeType& in, pyramid_data_model_agra_SystemComparativeType_c* out);
void from_c(const pyramid_data_model_agra_SystemComparativeType_c* in, pyramid::domain_model::agra::SystemComparativeType& out);

void to_c(const pyramid::domain_model::agra::SystemCharacteristicType& in, pyramid_data_model_agra_SystemCharacteristicType_c* out);
void from_c(const pyramid_data_model_agra_SystemCharacteristicType_c* in, pyramid::domain_model::agra::SystemCharacteristicType& out);

void to_c(const pyramid::domain_model::agra::PrioritizationListValueType& in, pyramid_data_model_agra_PrioritizationListValueType_c* out);
void from_c(const pyramid_data_model_agra_PrioritizationListValueType_c* in, pyramid::domain_model::agra::PrioritizationListValueType& out);

void to_c(const pyramid::domain_model::agra::PrioritizationType& in, pyramid_data_model_agra_PrioritizationType_c* out);
void from_c(const pyramid_data_model_agra_PrioritizationType_c* in, pyramid::domain_model::agra::PrioritizationType& out);

void to_c(const pyramid::domain_model::agra::RequirementTaxonomyType& in, pyramid_data_model_agra_RequirementTaxonomyType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTaxonomyType_c* in, pyramid::domain_model::agra::RequirementTaxonomyType& out);

void to_c(const pyramid::domain_model::agra::EntityFilterType& in, pyramid_data_model_agra_EntityFilterType_c* out);
void from_c(const pyramid_data_model_agra_EntityFilterType_c* in, pyramid::domain_model::agra::EntityFilterType& out);

void to_c(const pyramid::domain_model::agra::DesignationFilterType& in, pyramid_data_model_agra_DesignationFilterType_c* out);
void from_c(const pyramid_data_model_agra_DesignationFilterType_c* in, pyramid::domain_model::agra::DesignationFilterType& out);

void to_c(const pyramid::domain_model::agra::EntityComparativeType& in, pyramid_data_model_agra_EntityComparativeType_c* out);
void from_c(const pyramid_data_model_agra_EntityComparativeType_c* in, pyramid::domain_model::agra::EntityComparativeType& out);

void to_c(const pyramid::domain_model::agra::EntityCharacteristicType& in, pyramid_data_model_agra_EntityCharacteristicType_c* out);
void from_c(const pyramid_data_model_agra_EntityCharacteristicType_c* in, pyramid::domain_model::agra::EntityCharacteristicType& out);

void to_c(const pyramid::domain_model::agra::RequirementMetadataType& in, pyramid_data_model_agra_RequirementMetadataType_c* out);
void from_c(const pyramid_data_model_agra_RequirementMetadataType_c* in, pyramid::domain_model::agra::RequirementMetadataType& out);

void to_c(const pyramid::domain_model::agra::TraceabilityType& in, pyramid_data_model_agra_TraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_TraceabilityType_c* in, pyramid::domain_model::agra::TraceabilityType& out);

void to_c(const pyramid::domain_model::agra::CollectionDeckTraceabilityType& in, pyramid_data_model_agra_CollectionDeckTraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_CollectionDeckTraceabilityType_c* in, pyramid::domain_model::agra::CollectionDeckTraceabilityType& out);

void to_c(const pyramid::domain_model::agra::ACTDF_TraceabilityType& in, pyramid_data_model_agra_ACTDF_TraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_ACTDF_TraceabilityType_c* in, pyramid::domain_model::agra::ACTDF_TraceabilityType& out);

void to_c(const pyramid::domain_model::agra::ACTDF_CollectionPlanType& in, pyramid_data_model_agra_ACTDF_CollectionPlanType_c* out);
void from_c(const pyramid_data_model_agra_ACTDF_CollectionPlanType_c* in, pyramid::domain_model::agra::ACTDF_CollectionPlanType& out);

void to_c(const pyramid::domain_model::agra::ACTDF_TaskID_Type& in, pyramid_data_model_agra_ACTDF_TaskID_Type_c* out);
void from_c(const pyramid_data_model_agra_ACTDF_TaskID_Type_c* in, pyramid::domain_model::agra::ACTDF_TaskID_Type& out);

void to_c(const pyramid::domain_model::agra::EEI_ID_Type& in, pyramid_data_model_agra_EEI_ID_Type_c* out);
void from_c(const pyramid_data_model_agra_EEI_ID_Type_c* in, pyramid::domain_model::agra::EEI_ID_Type& out);

void to_c(const pyramid::domain_model::agra::ATO_TraceabilityType& in, pyramid_data_model_agra_ATO_TraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_ATO_TraceabilityType_c* in, pyramid::domain_model::agra::ATO_TraceabilityType& out);

void to_c(const pyramid::domain_model::agra::AOCO_TraceabilityType& in, pyramid_data_model_agra_AOCO_TraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_AOCO_TraceabilityType_c* in, pyramid::domain_model::agra::AOCO_TraceabilityType& out);

void to_c(const pyramid::domain_model::agra::CS_STO_TraceabilityType& in, pyramid_data_model_agra_CS_STO_TraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_CS_STO_TraceabilityType_c* in, pyramid::domain_model::agra::CS_STO_TraceabilityType& out);

void to_c(const pyramid::domain_model::agra::RemarksType& in, pyramid_data_model_agra_RemarksType_c* out);
void from_c(const pyramid_data_model_agra_RemarksType_c* in, pyramid::domain_model::agra::RemarksType& out);

void to_c(const pyramid::domain_model::agra::MA_ActionStatusMT& in, pyramid_data_model_agra_MA_ActionStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_ActionStatusMT_c* in, pyramid::domain_model::agra::MA_ActionStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_ActionStatusMDT& in, pyramid_data_model_agra_MA_ActionStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_ActionStatusMDT_c* in, pyramid::domain_model::agra::MA_ActionStatusMDT& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementExecutionStatusDetailsType& in, pyramid_data_model_agra_MA_RequirementExecutionStatusDetailsType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementExecutionStatusDetailsType_c* in, pyramid::domain_model::agra::MA_RequirementExecutionStatusDetailsType& out);

void to_c(const pyramid::domain_model::agra::MA_PackageSystemType& in, pyramid_data_model_agra_MA_PackageSystemType_c* out);
void from_c(const pyramid_data_model_agra_MA_PackageSystemType_c* in, pyramid::domain_model::agra::MA_PackageSystemType& out);

void to_c(const pyramid::domain_model::agra::CannotComplyType& in, pyramid_data_model_agra_CannotComplyType_c* out);
void from_c(const pyramid_data_model_agra_CannotComplyType_c* in, pyramid::domain_model::agra::CannotComplyType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementStatusTraceabilityType& in, pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementStatusTraceabilityType_c* in, pyramid::domain_model::agra::MA_RequirementStatusTraceabilityType& out);

void to_c(const pyramid::domain_model::agra::MA_PlansReferenceType& in, pyramid_data_model_agra_MA_PlansReferenceType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlansReferenceType_c* in, pyramid::domain_model::agra::MA_PlansReferenceType& out);

void to_c(const pyramid::domain_model::agra::MA_PlansReferenceBaseType& in, pyramid_data_model_agra_MA_PlansReferenceBaseType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlansReferenceBaseType_c* in, pyramid::domain_model::agra::MA_PlansReferenceBaseType& out);

void to_c(const pyramid::domain_model::agra::TaskPlanID_Type& in, pyramid_data_model_agra_TaskPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_TaskPlanID_Type_c* in, pyramid::domain_model::agra::TaskPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanID_Type& in, pyramid_data_model_agra_OrbitPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanID_Type_c* in, pyramid::domain_model::agra::OrbitPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::OrbitActivityPlanID_Type& in, pyramid_data_model_agra_OrbitActivityPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_OrbitActivityPlanID_Type_c* in, pyramid::domain_model::agra::OrbitActivityPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::RoutePlanID_Type& in, pyramid_data_model_agra_RoutePlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanID_Type_c* in, pyramid::domain_model::agra::RoutePlanID_Type& out);

void to_c(const pyramid::domain_model::agra::RouteActivityPlanID_Type& in, pyramid_data_model_agra_RouteActivityPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_RouteActivityPlanID_Type_c* in, pyramid::domain_model::agra::RouteActivityPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::ActivityPlanID_Type& in, pyramid_data_model_agra_ActivityPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_ActivityPlanID_Type_c* in, pyramid::domain_model::agra::ActivityPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::EffectPlanID_Type& in, pyramid_data_model_agra_EffectPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_EffectPlanID_Type_c* in, pyramid::domain_model::agra::EffectPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::ActionPlanID_Type& in, pyramid_data_model_agra_ActionPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_ActionPlanID_Type_c* in, pyramid::domain_model::agra::ActionPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::ResponsePlanID_Type& in, pyramid_data_model_agra_ResponsePlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_ResponsePlanID_Type_c* in, pyramid::domain_model::agra::ResponsePlanID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_OpPlanID_Type& in, pyramid_data_model_agra_MA_OpPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_MA_OpPlanID_Type_c* in, pyramid::domain_model::agra::MA_OpPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::MissionPlanID_Type& in, pyramid_data_model_agra_MissionPlanID_Type_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanID_Type_c* in, pyramid::domain_model::agra::MissionPlanID_Type& out);

void to_c(const pyramid::domain_model::agra::PlannedActivityID_Type& in, pyramid_data_model_agra_PlannedActivityID_Type_c* out);
void from_c(const pyramid_data_model_agra_PlannedActivityID_Type_c* in, pyramid::domain_model::agra::PlannedActivityID_Type& out);

void to_c(const pyramid::domain_model::agra::ActivityID_Type& in, pyramid_data_model_agra_ActivityID_Type_c* out);
void from_c(const pyramid_data_model_agra_ActivityID_Type_c* in, pyramid::domain_model::agra::ActivityID_Type& out);

void to_c(const pyramid::domain_model::agra::MetricsType& in, pyramid_data_model_agra_MetricsType_c* out);
void from_c(const pyramid_data_model_agra_MetricsType_c* in, pyramid::domain_model::agra::MetricsType& out);

void to_c(const pyramid::domain_model::agra::EnduranceType& in, pyramid_data_model_agra_EnduranceType_c* out);
void from_c(const pyramid_data_model_agra_EnduranceType_c* in, pyramid::domain_model::agra::EnduranceType& out);

void to_c(const pyramid::domain_model::agra::EnduranceBaseType& in, pyramid_data_model_agra_EnduranceBaseType_c* out);
void from_c(const pyramid_data_model_agra_EnduranceBaseType_c* in, pyramid::domain_model::agra::EnduranceBaseType& out);

void to_c(const pyramid::domain_model::agra::EnduranceFootprintType& in, pyramid_data_model_agra_EnduranceFootprintType_c* out);
void from_c(const pyramid_data_model_agra_EnduranceFootprintType_c* in, pyramid::domain_model::agra::EnduranceFootprintType& out);

void to_c(const pyramid::domain_model::agra::BoundaryType& in, pyramid_data_model_agra_BoundaryType_c* out);
void from_c(const pyramid_data_model_agra_BoundaryType_c* in, pyramid::domain_model::agra::BoundaryType& out);

void to_c(const pyramid::domain_model::agra::ExpendableType& in, pyramid_data_model_agra_ExpendableType_c* out);
void from_c(const pyramid_data_model_agra_ExpendableType_c* in, pyramid::domain_model::agra::ExpendableType& out);

void to_c(const pyramid::domain_model::agra::WeaponStoreType& in, pyramid_data_model_agra_WeaponStoreType_c* out);
void from_c(const pyramid_data_model_agra_WeaponStoreType_c* in, pyramid::domain_model::agra::WeaponStoreType& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalPolicyMT& in, pyramid_data_model_agra_MA_ApprovalPolicyMT_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalPolicyMT_c* in, pyramid::domain_model::agra::MA_ApprovalPolicyMT& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalPolicyMDT& in, pyramid_data_model_agra_MA_ApprovalPolicyMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalPolicyMDT_c* in, pyramid::domain_model::agra::MA_ApprovalPolicyMDT& out);

void to_c(const pyramid::domain_model::agra::DataRecordBaseType& in, pyramid_data_model_agra_DataRecordBaseType_c* out);
void from_c(const pyramid_data_model_agra_DataRecordBaseType_c* in, pyramid::domain_model::agra::DataRecordBaseType& out);

void to_c(const pyramid::domain_model::agra::DataRecordInstanceID_Type& in, pyramid_data_model_agra_DataRecordInstanceID_Type_c* out);
void from_c(const pyramid_data_model_agra_DataRecordInstanceID_Type_c* in, pyramid::domain_model::agra::DataRecordInstanceID_Type& out);

void to_c(const pyramid::domain_model::agra::ApprovalPolicyID_Type& in, pyramid_data_model_agra_ApprovalPolicyID_Type_c* out);
void from_c(const pyramid_data_model_agra_ApprovalPolicyID_Type_c* in, pyramid::domain_model::agra::ApprovalPolicyID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_PlanPolicyType& in, pyramid_data_model_agra_MA_PlanPolicyType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanPolicyType_c* in, pyramid::domain_model::agra::MA_PlanPolicyType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanPolicyApplicablePlanType& in, pyramid_data_model_agra_MA_PlanPolicyApplicablePlanType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanPolicyApplicablePlanType_c* in, pyramid::domain_model::agra::MA_PlanPolicyApplicablePlanType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanPartsType& in, pyramid_data_model_agra_MA_PlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanPartsType_c* in, pyramid::domain_model::agra::MA_PlanPartsType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanPartsBaseType& in, pyramid_data_model_agra_MA_PlanPartsBaseType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanPartsBaseType_c* in, pyramid::domain_model::agra::MA_PlanPartsBaseType& out);

void to_c(const pyramid::domain_model::agra::EffectPlanPartsType& in, pyramid_data_model_agra_EffectPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_EffectPlanPartsType_c* in, pyramid::domain_model::agra::EffectPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::ActionPlanPartsType& in, pyramid_data_model_agra_ActionPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_ActionPlanPartsType_c* in, pyramid::domain_model::agra::ActionPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::MA_TaskPlanPartsType& in, pyramid_data_model_agra_MA_TaskPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskPlanPartsType_c* in, pyramid::domain_model::agra::MA_TaskPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::ResponsePlanPartsType& in, pyramid_data_model_agra_ResponsePlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_ResponsePlanPartsType_c* in, pyramid::domain_model::agra::ResponsePlanPartsType& out);

void to_c(const pyramid::domain_model::agra::MA_OpPlanPartsType& in, pyramid_data_model_agra_MA_OpPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_MA_OpPlanPartsType_c* in, pyramid::domain_model::agra::MA_OpPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::OpPointCategoriesType& in, pyramid_data_model_agra_OpPointCategoriesType_c* out);
void from_c(const pyramid_data_model_agra_OpPointCategoriesType_c* in, pyramid::domain_model::agra::OpPointCategoriesType& out);

void to_c(const pyramid::domain_model::agra::MA_ActivityPlanPartsType& in, pyramid_data_model_agra_MA_ActivityPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_MA_ActivityPlanPartsType_c* in, pyramid::domain_model::agra::MA_ActivityPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanPartsType& in, pyramid_data_model_agra_RoutePlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanPartsType_c* in, pyramid::domain_model::agra::RoutePlanPartsType& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanPartsType& in, pyramid_data_model_agra_OrbitPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanPartsType_c* in, pyramid::domain_model::agra::OrbitPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::AutonomousPlanningConstrainingPlansType& in, pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c* out);
void from_c(const pyramid_data_model_agra_AutonomousPlanningConstrainingPlansType_c* in, pyramid::domain_model::agra::AutonomousPlanningConstrainingPlansType& out);

void to_c(const pyramid::domain_model::agra::ConstrainingPlanPartsType& in, pyramid_data_model_agra_ConstrainingPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainingPlanPartsType_c* in, pyramid::domain_model::agra::ConstrainingPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::PlanPartsType& in, pyramid_data_model_agra_PlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_PlanPartsType_c* in, pyramid::domain_model::agra::PlanPartsType& out);

void to_c(const pyramid::domain_model::agra::PlanPartsBaseType& in, pyramid_data_model_agra_PlanPartsBaseType_c* out);
void from_c(const pyramid_data_model_agra_PlanPartsBaseType_c* in, pyramid::domain_model::agra::PlanPartsBaseType& out);

void to_c(const pyramid::domain_model::agra::TaskPlanPartsType& in, pyramid_data_model_agra_TaskPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_TaskPlanPartsType_c* in, pyramid::domain_model::agra::TaskPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::ActivityPlanPartsType& in, pyramid_data_model_agra_ActivityPlanPartsType_c* out);
void from_c(const pyramid_data_model_agra_ActivityPlanPartsType_c* in, pyramid::domain_model::agra::ActivityPlanPartsType& out);

void to_c(const pyramid::domain_model::agra::CommAllocationPartsType& in, pyramid_data_model_agra_CommAllocationPartsType_c* out);
void from_c(const pyramid_data_model_agra_CommAllocationPartsType_c* in, pyramid::domain_model::agra::CommAllocationPartsType& out);

void to_c(const pyramid::domain_model::agra::AutonomousPlanningOtherSystemConstrainingPlansType& in, pyramid_data_model_agra_AutonomousPlanningOtherSystemConstrainingPlansType_c* out);
void from_c(const pyramid_data_model_agra_AutonomousPlanningOtherSystemConstrainingPlansType_c* in, pyramid::domain_model::agra::AutonomousPlanningOtherSystemConstrainingPlansType& out);

void to_c(const pyramid::domain_model::agra::ApprovalPolicyBaseType& in, pyramid_data_model_agra_ApprovalPolicyBaseType_c* out);
void from_c(const pyramid_data_model_agra_ApprovalPolicyBaseType_c* in, pyramid::domain_model::agra::ApprovalPolicyBaseType& out);

void to_c(const pyramid::domain_model::agra::DefaultResponseType& in, pyramid_data_model_agra_DefaultResponseType_c* out);
void from_c(const pyramid_data_model_agra_DefaultResponseType_c* in, pyramid::domain_model::agra::DefaultResponseType& out);

void to_c(const pyramid::domain_model::agra::ByResultPolicyType& in, pyramid_data_model_agra_ByResultPolicyType_c* out);
void from_c(const pyramid_data_model_agra_ByResultPolicyType_c* in, pyramid::domain_model::agra::ByResultPolicyType& out);

void to_c(const pyramid::domain_model::agra::PlanningByResultTriggerType& in, pyramid_data_model_agra_PlanningByResultTriggerType_c* out);
void from_c(const pyramid_data_model_agra_PlanningByResultTriggerType_c* in, pyramid::domain_model::agra::PlanningByResultTriggerType& out);

void to_c(const pyramid::domain_model::agra::PlanningByResultTriggerType_ReplanRequired_List& in, pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c* out);
void from_c(const pyramid_data_model_agra_PlanningByResultTriggerType_ReplanRequired_List_c* in, pyramid::domain_model::agra::PlanningByResultTriggerType_ReplanRequired_List& out);

void to_c(const pyramid::domain_model::agra::PlanVulnerabilityType& in, pyramid_data_model_agra_PlanVulnerabilityType_c* out);
void from_c(const pyramid_data_model_agra_PlanVulnerabilityType_c* in, pyramid::domain_model::agra::PlanVulnerabilityType& out);

void to_c(const pyramid::domain_model::agra::ThreatVulnerabilityByCapabilityType& in, pyramid_data_model_agra_ThreatVulnerabilityByCapabilityType_c* out);
void from_c(const pyramid_data_model_agra_ThreatVulnerabilityByCapabilityType_c* in, pyramid::domain_model::agra::ThreatVulnerabilityByCapabilityType& out);

void to_c(const pyramid::domain_model::agra::ThresholdVulnerabilityType& in, pyramid_data_model_agra_ThresholdVulnerabilityType_c* out);
void from_c(const pyramid_data_model_agra_ThresholdVulnerabilityType_c* in, pyramid::domain_model::agra::ThresholdVulnerabilityType& out);

void to_c(const pyramid::domain_model::agra::VulnerabilityLevelsCombinedType& in, pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c* out);
void from_c(const pyramid_data_model_agra_VulnerabilityLevelsCombinedType_c* in, pyramid::domain_model::agra::VulnerabilityLevelsCombinedType& out);

void to_c(const pyramid::domain_model::agra::RequirementTriggerType& in, pyramid_data_model_agra_RequirementTriggerType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTriggerType_c* in, pyramid::domain_model::agra::RequirementTriggerType& out);

void to_c(const pyramid::domain_model::agra::RankCompareType& in, pyramid_data_model_agra_RankCompareType_c* out);
void from_c(const pyramid_data_model_agra_RankCompareType_c* in, pyramid::domain_model::agra::RankCompareType& out);

void to_c(const pyramid::domain_model::agra::RequirementTaxonomyDetailedType& in, pyramid_data_model_agra_RequirementTaxonomyDetailedType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTaxonomyDetailedType_c* in, pyramid::domain_model::agra::RequirementTaxonomyDetailedType& out);

void to_c(const pyramid::domain_model::agra::ByTriggerPolicyType& in, pyramid_data_model_agra_ByTriggerPolicyType_c* out);
void from_c(const pyramid_data_model_agra_ByTriggerPolicyType_c* in, pyramid::domain_model::agra::ByTriggerPolicyType& out);

void to_c(const pyramid::domain_model::agra::PlanningByCaseTriggerType& in, pyramid_data_model_agra_PlanningByCaseTriggerType_c* out);
void from_c(const pyramid_data_model_agra_PlanningByCaseTriggerType_c* in, pyramid::domain_model::agra::PlanningByCaseTriggerType& out);

void to_c(const pyramid::domain_model::agra::CommsLostTriggerDataType& in, pyramid_data_model_agra_CommsLostTriggerDataType_c* out);
void from_c(const pyramid_data_model_agra_CommsLostTriggerDataType_c* in, pyramid::domain_model::agra::CommsLostTriggerDataType& out);

void to_c(const pyramid::domain_model::agra::ThresholdOffRouteTriggerDataType& in, pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c* out);
void from_c(const pyramid_data_model_agra_ThresholdOffRouteTriggerDataType_c* in, pyramid::domain_model::agra::ThresholdOffRouteTriggerDataType& out);

void to_c(const pyramid::domain_model::agra::SystemStateFilterType& in, pyramid_data_model_agra_SystemStateFilterType_c* out);
void from_c(const pyramid_data_model_agra_SystemStateFilterType_c* in, pyramid::domain_model::agra::SystemStateFilterType& out);

void to_c(const pyramid::domain_model::agra::RequirementFailedTriggerType& in, pyramid_data_model_agra_RequirementFailedTriggerType_c* out);
void from_c(const pyramid_data_model_agra_RequirementFailedTriggerType_c* in, pyramid::domain_model::agra::RequirementFailedTriggerType& out);

void to_c(const pyramid::domain_model::agra::ZoneViolationTriggerDataType& in, pyramid_data_model_agra_ZoneViolationTriggerDataType_c* out);
void from_c(const pyramid_data_model_agra_ZoneViolationTriggerDataType_c* in, pyramid::domain_model::agra::ZoneViolationTriggerDataType& out);

void to_c(const pyramid::domain_model::agra::SatelliteEnduranceType& in, pyramid_data_model_agra_SatelliteEnduranceType_c* out);
void from_c(const pyramid_data_model_agra_SatelliteEnduranceType_c* in, pyramid::domain_model::agra::SatelliteEnduranceType& out);

void to_c(const pyramid::domain_model::agra::OrbitalManeuverDetailsBaseType& in, pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalManeuverDetailsBaseType_c* in, pyramid::domain_model::agra::OrbitalManeuverDetailsBaseType& out);

void to_c(const pyramid::domain_model::agra::OrbitalDeltaVelocity_B_Type& in, pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c* out);
void from_c(const pyramid_data_model_agra_OrbitalDeltaVelocity_B_Type_c* in, pyramid::domain_model::agra::OrbitalDeltaVelocity_B_Type& out);

void to_c(const pyramid::domain_model::agra::MA_PlanActivationPolicyType& in, pyramid_data_model_agra_MA_PlanActivationPolicyType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanActivationPolicyType_c* in, pyramid::domain_model::agra::MA_PlanActivationPolicyType& out);

void to_c(const pyramid::domain_model::agra::MissionPlanActivationSettingType& in, pyramid_data_model_agra_MissionPlanActivationSettingType_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanActivationSettingType_c* in, pyramid::domain_model::agra::MissionPlanActivationSettingType& out);

void to_c(const pyramid::domain_model::agra::SubPlanActivationSettingType& in, pyramid_data_model_agra_SubPlanActivationSettingType_c* out);
void from_c(const pyramid_data_model_agra_SubPlanActivationSettingType_c* in, pyramid::domain_model::agra::SubPlanActivationSettingType& out);

void to_c(const pyramid::domain_model::agra::RequirementExecutionPolicyType& in, pyramid_data_model_agra_RequirementExecutionPolicyType_c* out);
void from_c(const pyramid_data_model_agra_RequirementExecutionPolicyType_c* in, pyramid::domain_model::agra::RequirementExecutionPolicyType& out);

void to_c(const pyramid::domain_model::agra::ByRequirementPolicyType& in, pyramid_data_model_agra_ByRequirementPolicyType_c* out);
void from_c(const pyramid_data_model_agra_ByRequirementPolicyType_c* in, pyramid::domain_model::agra::ByRequirementPolicyType& out);

void to_c(const pyramid::domain_model::agra::TimedZoneType& in, pyramid_data_model_agra_TimedZoneType_c* out);
void from_c(const pyramid_data_model_agra_TimedZoneType_c* in, pyramid::domain_model::agra::TimedZoneType& out);

void to_c(const pyramid::domain_model::agra::ScheduleType& in, pyramid_data_model_agra_ScheduleType_c* out);
void from_c(const pyramid_data_model_agra_ScheduleType_c* in, pyramid::domain_model::agra::ScheduleType& out);

void to_c(const pyramid::domain_model::agra::ScheduleType_TimeSpan_List& in, pyramid_data_model_agra_ScheduleType_TimeSpan_List_c* out);
void from_c(const pyramid_data_model_agra_ScheduleType_TimeSpan_List_c* in, pyramid::domain_model::agra::ScheduleType_TimeSpan_List& out);

void to_c(const pyramid::domain_model::agra::ScheduleType_WeekdayInterval_List& in, pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c* out);
void from_c(const pyramid_data_model_agra_ScheduleType_WeekdayInterval_List_c* in, pyramid::domain_model::agra::ScheduleType_WeekdayInterval_List& out);

void to_c(const pyramid::domain_model::agra::MissionTraceabilityType& in, pyramid_data_model_agra_MissionTraceabilityType_c* out);
void from_c(const pyramid_data_model_agra_MissionTraceabilityType_c* in, pyramid::domain_model::agra::MissionTraceabilityType& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestMT& in, pyramid_data_model_agra_MA_ApprovalRequestMT_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestMT_c* in, pyramid::domain_model::agra::MA_ApprovalRequestMT& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestMDT& in, pyramid_data_model_agra_MA_ApprovalRequestMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestMDT_c* in, pyramid::domain_model::agra::MA_ApprovalRequestMDT& out);

void to_c(const pyramid::domain_model::agra::RequestBaseType& in, pyramid_data_model_agra_RequestBaseType_c* out);
void from_c(const pyramid_data_model_agra_RequestBaseType_c* in, pyramid::domain_model::agra::RequestBaseType& out);

void to_c(const pyramid::domain_model::agra::RequestID_Type& in, pyramid_data_model_agra_RequestID_Type_c* out);
void from_c(const pyramid_data_model_agra_RequestID_Type_c* in, pyramid::domain_model::agra::RequestID_Type& out);

void to_c(const pyramid::domain_model::agra::OperatorRoleType& in, pyramid_data_model_agra_OperatorRoleType_c* out);
void from_c(const pyramid_data_model_agra_OperatorRoleType_c* in, pyramid::domain_model::agra::OperatorRoleType& out);

void to_c(const pyramid::domain_model::agra::OperatorRoleID_Type& in, pyramid_data_model_agra_OperatorRoleID_Type_c* out);
void from_c(const pyramid_data_model_agra_OperatorRoleID_Type_c* in, pyramid::domain_model::agra::OperatorRoleID_Type& out);

void to_c(const pyramid::domain_model::agra::SystemServiceType& in, pyramid_data_model_agra_SystemServiceType_c* out);
void from_c(const pyramid_data_model_agra_SystemServiceType_c* in, pyramid::domain_model::agra::SystemServiceType& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestPolicyReferenceType& in, pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestPolicyReferenceType_c* in, pyramid::domain_model::agra::MA_ApprovalRequestPolicyReferenceType& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestItemReferenceType& in, pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_c* in, pyramid::domain_model::agra::MA_ApprovalRequestItemReferenceType& out);

void to_c(const pyramid::domain_model::agra::PlanReferenceID_ChoiceType& in, pyramid_data_model_agra_PlanReferenceID_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_PlanReferenceID_ChoiceType_c* in, pyramid::domain_model::agra::PlanReferenceID_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::CommScheduleAllocationID_Type& in, pyramid_data_model_agra_CommScheduleAllocationID_Type_c* out);
void from_c(const pyramid_data_model_agra_CommScheduleAllocationID_Type_c* in, pyramid::domain_model::agra::CommScheduleAllocationID_Type& out);

void to_c(const pyramid::domain_model::agra::ApprovalRequestItemType& in, pyramid_data_model_agra_ApprovalRequestItemType_c* out);
void from_c(const pyramid_data_model_agra_ApprovalRequestItemType_c* in, pyramid::domain_model::agra::ApprovalRequestItemType& out);

void to_c(const pyramid::domain_model::agra::DMPI_ID_Type& in, pyramid_data_model_agra_DMPI_ID_Type_c* out);
void from_c(const pyramid_data_model_agra_DMPI_ID_Type_c* in, pyramid::domain_model::agra::DMPI_ID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationCommandType& in, pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationCommandType_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationCommandType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationDetailsType& in, pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationDetailsType_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationDetailsType& out);

void to_c(const pyramid::domain_model::agra::MissionPlanActivationType& in, pyramid_data_model_agra_MissionPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanActivationType_c* in, pyramid::domain_model::agra::MissionPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanSubplanActivationType& in, pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanSubplanActivationType_c* in, pyramid::domain_model::agra::MA_MissionPlanSubplanActivationType& out);

void to_c(const pyramid::domain_model::agra::TaskPlanActivationType& in, pyramid_data_model_agra_TaskPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_TaskPlanActivationType_c* in, pyramid::domain_model::agra::TaskPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanActivationType& in, pyramid_data_model_agra_RoutePlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanActivationType_c* in, pyramid::domain_model::agra::RoutePlanActivationType& out);

void to_c(const pyramid::domain_model::agra::ActivationPathSegmentType& in, pyramid_data_model_agra_ActivationPathSegmentType_c* out);
void from_c(const pyramid_data_model_agra_ActivationPathSegmentType_c* in, pyramid::domain_model::agra::ActivationPathSegmentType& out);

void to_c(const pyramid::domain_model::agra::PathID_Type& in, pyramid_data_model_agra_PathID_Type_c* out);
void from_c(const pyramid_data_model_agra_PathID_Type_c* in, pyramid::domain_model::agra::PathID_Type& out);

void to_c(const pyramid::domain_model::agra::SegmentID_Type& in, pyramid_data_model_agra_SegmentID_Type_c* out);
void from_c(const pyramid_data_model_agra_SegmentID_Type_c* in, pyramid::domain_model::agra::SegmentID_Type& out);

void to_c(const pyramid::domain_model::agra::RouteActivityPlanActivationType& in, pyramid_data_model_agra_RouteActivityPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_RouteActivityPlanActivationType_c* in, pyramid::domain_model::agra::RouteActivityPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanActivationType& in, pyramid_data_model_agra_OrbitPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanActivationType_c* in, pyramid::domain_model::agra::OrbitPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::ActivationOrbitSequenceType& in, pyramid_data_model_agra_ActivationOrbitSequenceType_c* out);
void from_c(const pyramid_data_model_agra_ActivationOrbitSequenceType_c* in, pyramid::domain_model::agra::ActivationOrbitSequenceType& out);

void to_c(const pyramid::domain_model::agra::OrbitKinematicsSequenceID_Type& in, pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c* out);
void from_c(const pyramid_data_model_agra_OrbitKinematicsSequenceID_Type_c* in, pyramid::domain_model::agra::OrbitKinematicsSequenceID_Type& out);

void to_c(const pyramid::domain_model::agra::OrbitManeuverSegmentID_Type& in, pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c* out);
void from_c(const pyramid_data_model_agra_OrbitManeuverSegmentID_Type_c* in, pyramid::domain_model::agra::OrbitManeuverSegmentID_Type& out);

void to_c(const pyramid::domain_model::agra::OrbitActivityPlanActivationType& in, pyramid_data_model_agra_OrbitActivityPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_OrbitActivityPlanActivationType_c* in, pyramid::domain_model::agra::OrbitActivityPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::ActivityPlanActivationType& in, pyramid_data_model_agra_ActivityPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_ActivityPlanActivationType_c* in, pyramid::domain_model::agra::ActivityPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::EffectPlanActivationType& in, pyramid_data_model_agra_EffectPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_EffectPlanActivationType_c* in, pyramid::domain_model::agra::EffectPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::ActionPlanActivationType& in, pyramid_data_model_agra_ActionPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_ActionPlanActivationType_c* in, pyramid::domain_model::agra::ActionPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::ResponsePlanActivationType& in, pyramid_data_model_agra_ResponsePlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_ResponsePlanActivationType_c* in, pyramid::domain_model::agra::ResponsePlanActivationType& out);

void to_c(const pyramid::domain_model::agra::MA_OpPlanActivationType& in, pyramid_data_model_agra_MA_OpPlanActivationType_c* out);
void from_c(const pyramid_data_model_agra_MA_OpPlanActivationType_c* in, pyramid::domain_model::agra::MA_OpPlanActivationType& out);

void to_c(const pyramid::domain_model::agra::MA_ExecutionPlanSetActivationType& in, pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c* out);
void from_c(const pyramid_data_model_agra_MA_ExecutionPlanSetActivationType_c* in, pyramid::domain_model::agra::MA_ExecutionPlanSetActivationType& out);

void to_c(const pyramid::domain_model::agra::ExecutionPlanSetID_Type& in, pyramid_data_model_agra_ExecutionPlanSetID_Type_c* out);
void from_c(const pyramid_data_model_agra_ExecutionPlanSetID_Type_c* in, pyramid::domain_model::agra::ExecutionPlanSetID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List& in, pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List_c* in, pyramid::domain_model::agra::MA_ApprovalRequestItemReferenceType_MissionPlanActivationApproval_List& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestStatusMT& in, pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c* in, pyramid::domain_model::agra::MA_ApprovalRequestStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_ApprovalRequestStatusMDT& in, pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_ApprovalRequestStatusMDT_c* in, pyramid::domain_model::agra::MA_ApprovalRequestStatusMDT& out);

void to_c(const pyramid::domain_model::agra::RequestStatusBaseType& in, pyramid_data_model_agra_RequestStatusBaseType_c* out);
void from_c(const pyramid_data_model_agra_RequestStatusBaseType_c* in, pyramid::domain_model::agra::RequestStatusBaseType& out);

void to_c(const pyramid::domain_model::agra::OperatorID_Type& in, pyramid_data_model_agra_OperatorID_Type_c* out);
void from_c(const pyramid_data_model_agra_OperatorID_Type_c* in, pyramid::domain_model::agra::OperatorID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanMT& in, pyramid_data_model_agra_MA_MissionPlanMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanMT_c* in, pyramid::domain_model::agra::MA_MissionPlanMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanMDT& in, pyramid_data_model_agra_MA_MissionPlanMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanMDT& out);

void to_c(const pyramid::domain_model::agra::MissionPlanCommandID_ChoiceType& in, pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanCommandID_ChoiceType_c* in, pyramid::domain_model::agra::MissionPlanCommandID_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::MissionPlanCommandID_Type& in, pyramid_data_model_agra_MissionPlanCommandID_Type_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanCommandID_Type_c* in, pyramid::domain_model::agra::MissionPlanCommandID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_PlanApplicabilityType& in, pyramid_data_model_agra_MA_PlanApplicabilityType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanApplicabilityType_c* in, pyramid::domain_model::agra::MA_PlanApplicabilityType& out);

void to_c(const pyramid::domain_model::agra::ExecutionSequenceType& in, pyramid_data_model_agra_ExecutionSequenceType_c* out);
void from_c(const pyramid_data_model_agra_ExecutionSequenceType_c* in, pyramid::domain_model::agra::ExecutionSequenceType& out);

void to_c(const pyramid::domain_model::agra::ExecutionSequencePlanSetsType& in, pyramid_data_model_agra_ExecutionSequencePlanSetsType_c* out);
void from_c(const pyramid_data_model_agra_ExecutionSequencePlanSetsType_c* in, pyramid::domain_model::agra::ExecutionSequencePlanSetsType& out);

void to_c(const pyramid::domain_model::agra::RouteExecutionPlanSetType& in, pyramid_data_model_agra_RouteExecutionPlanSetType_c* out);
void from_c(const pyramid_data_model_agra_RouteExecutionPlanSetType_c* in, pyramid::domain_model::agra::RouteExecutionPlanSetType& out);

void to_c(const pyramid::domain_model::agra::ExecutionPlanSetBaseType& in, pyramid_data_model_agra_ExecutionPlanSetBaseType_c* out);
void from_c(const pyramid_data_model_agra_ExecutionPlanSetBaseType_c* in, pyramid::domain_model::agra::ExecutionPlanSetBaseType& out);

void to_c(const pyramid::domain_model::agra::OrbitExecutionPlanSetType& in, pyramid_data_model_agra_OrbitExecutionPlanSetType_c* out);
void from_c(const pyramid_data_model_agra_OrbitExecutionPlanSetType_c* in, pyramid::domain_model::agra::OrbitExecutionPlanSetType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanInputsType& in, pyramid_data_model_agra_MA_MissionPlanInputsType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanInputsType_c* in, pyramid::domain_model::agra::MA_MissionPlanInputsType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanInputsCoreType& in, pyramid_data_model_agra_MA_PlanInputsCoreType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanInputsCoreType_c* in, pyramid::domain_model::agra::MA_PlanInputsCoreType& out);

void to_c(const pyramid::domain_model::agra::PlanningProcessID_Type& in, pyramid_data_model_agra_PlanningProcessID_Type_c* out);
void from_c(const pyramid_data_model_agra_PlanningProcessID_Type_c* in, pyramid::domain_model::agra::PlanningProcessID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_ReplanReasonType& in, pyramid_data_model_agra_MA_ReplanReasonType_c* out);
void from_c(const pyramid_data_model_agra_MA_ReplanReasonType_c* in, pyramid::domain_model::agra::MA_ReplanReasonType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningTriggerType& in, pyramid_data_model_agra_MA_PlanningTriggerType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningTriggerType_c* in, pyramid::domain_model::agra::MA_PlanningTriggerType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningByCaseTriggerType& in, pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningByCaseTriggerType_c* in, pyramid::domain_model::agra::MA_PlanningByCaseTriggerType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementTriggerType& in, pyramid_data_model_agra_MA_RequirementTriggerType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementTriggerType_c* in, pyramid::domain_model::agra::MA_RequirementTriggerType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementTaxonomyDetailedType& in, pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementTaxonomyDetailedType_c* in, pyramid::domain_model::agra::MA_RequirementTaxonomyDetailedType& out);

void to_c(const pyramid::domain_model::agra::MA_CapabilityTaxonomyType& in, pyramid_data_model_agra_MA_CapabilityTaxonomyType_c* out);
void from_c(const pyramid_data_model_agra_MA_CapabilityTaxonomyType_c* in, pyramid::domain_model::agra::MA_CapabilityTaxonomyType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementFailedTriggerType& in, pyramid_data_model_agra_MA_RequirementFailedTriggerType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementFailedTriggerType_c* in, pyramid::domain_model::agra::MA_RequirementFailedTriggerType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningByResultTriggerType& in, pyramid_data_model_agra_MA_PlanningByResultTriggerType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningByResultTriggerType_c* in, pyramid::domain_model::agra::MA_PlanningByResultTriggerType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningByResultTriggerType_ReplanRequired_List& in, pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningByResultTriggerType_ReplanRequired_List_c* in, pyramid::domain_model::agra::MA_PlanningByResultTriggerType_ReplanRequired_List& out);

void to_c(const pyramid::domain_model::agra::AutonomousPlanningActionID_Type& in, pyramid_data_model_agra_AutonomousPlanningActionID_Type_c* out);
void from_c(const pyramid_data_model_agra_AutonomousPlanningActionID_Type_c* in, pyramid::domain_model::agra::AutonomousPlanningActionID_Type& out);

void to_c(const pyramid::domain_model::agra::MissionContingencyAlertID_Type& in, pyramid_data_model_agra_MissionContingencyAlertID_Type_c* out);
void from_c(const pyramid_data_model_agra_MissionContingencyAlertID_Type_c* in, pyramid::domain_model::agra::MissionContingencyAlertID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_MissionEnvironmentConstraintType& in, pyramid_data_model_agra_MA_MissionEnvironmentConstraintType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionEnvironmentConstraintType_c* in, pyramid::domain_model::agra::MA_MissionEnvironmentConstraintType& out);

void to_c(const pyramid::domain_model::agra::ConstrainedEntityType& in, pyramid_data_model_agra_ConstrainedEntityType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainedEntityType_c* in, pyramid::domain_model::agra::ConstrainedEntityType& out);

void to_c(const pyramid::domain_model::agra::ConstrainedOpPointType& in, pyramid_data_model_agra_ConstrainedOpPointType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainedOpPointType_c* in, pyramid::domain_model::agra::ConstrainedOpPointType& out);

void to_c(const pyramid::domain_model::agra::OpPointCategoriesUniqueDataType& in, pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c* out);
void from_c(const pyramid_data_model_agra_OpPointCategoriesUniqueDataType_c* in, pyramid::domain_model::agra::OpPointCategoriesUniqueDataType& out);

void to_c(const pyramid::domain_model::agra::EmergencyReferencePointType& in, pyramid_data_model_agra_EmergencyReferencePointType_c* out);
void from_c(const pyramid_data_model_agra_EmergencyReferencePointType_c* in, pyramid::domain_model::agra::EmergencyReferencePointType& out);

void to_c(const pyramid::domain_model::agra::OpPointReferenceType& in, pyramid_data_model_agra_OpPointReferenceType_c* out);
void from_c(const pyramid_data_model_agra_OpPointReferenceType_c* in, pyramid::domain_model::agra::OpPointReferenceType& out);

void to_c(const pyramid::domain_model::agra::GeoLocatedStoredObjectType& in, pyramid_data_model_agra_GeoLocatedStoredObjectType_c* out);
void from_c(const pyramid_data_model_agra_GeoLocatedStoredObjectType_c* in, pyramid::domain_model::agra::GeoLocatedStoredObjectType& out);

void to_c(const pyramid::domain_model::agra::EntityDataType& in, pyramid_data_model_agra_EntityDataType_c* out);
void from_c(const pyramid_data_model_agra_EntityDataType_c* in, pyramid::domain_model::agra::EntityDataType& out);

void to_c(const pyramid::domain_model::agra::EntityMDT& in, pyramid_data_model_agra_EntityMDT_c* out);
void from_c(const pyramid_data_model_agra_EntityMDT_c* in, pyramid::domain_model::agra::EntityMDT& out);

void to_c(const pyramid::domain_model::agra::DateTimeSigmaType& in, pyramid_data_model_agra_DateTimeSigmaType_c* out);
void from_c(const pyramid_data_model_agra_DateTimeSigmaType_c* in, pyramid::domain_model::agra::DateTimeSigmaType& out);

void to_c(const pyramid::domain_model::agra::EntitySourceType& in, pyramid_data_model_agra_EntitySourceType_c* out);
void from_c(const pyramid_data_model_agra_EntitySourceType_c* in, pyramid::domain_model::agra::EntitySourceType& out);

void to_c(const pyramid::domain_model::agra::EntitySourceSpecificDataType& in, pyramid_data_model_agra_EntitySourceSpecificDataType_c* out);
void from_c(const pyramid_data_model_agra_EntitySourceSpecificDataType_c* in, pyramid::domain_model::agra::EntitySourceSpecificDataType& out);

void to_c(const pyramid::domain_model::agra::CorrelatedEntityID_Type& in, pyramid_data_model_agra_CorrelatedEntityID_Type_c* out);
void from_c(const pyramid_data_model_agra_CorrelatedEntityID_Type_c* in, pyramid::domain_model::agra::CorrelatedEntityID_Type& out);

void to_c(const pyramid::domain_model::agra::DropRestrictionType& in, pyramid_data_model_agra_DropRestrictionType_c* out);
void from_c(const pyramid_data_model_agra_DropRestrictionType_c* in, pyramid::domain_model::agra::DropRestrictionType& out);

void to_c(const pyramid::domain_model::agra::AssociatedDropRestrictMessageType& in, pyramid_data_model_agra_AssociatedDropRestrictMessageType_c* out);
void from_c(const pyramid_data_model_agra_AssociatedDropRestrictMessageType_c* in, pyramid::domain_model::agra::AssociatedDropRestrictMessageType& out);

void to_c(const pyramid::domain_model::agra::EntitySourceIdentifierType& in, pyramid_data_model_agra_EntitySourceIdentifierType_c* out);
void from_c(const pyramid_data_model_agra_EntitySourceIdentifierType_c* in, pyramid::domain_model::agra::EntitySourceIdentifierType& out);

void to_c(const pyramid::domain_model::agra::EOB_RecordID_Type& in, pyramid_data_model_agra_EOB_RecordID_Type_c* out);
void from_c(const pyramid_data_model_agra_EOB_RecordID_Type_c* in, pyramid::domain_model::agra::EOB_RecordID_Type& out);

void to_c(const pyramid::domain_model::agra::EntityExternalType& in, pyramid_data_model_agra_EntityExternalType_c* out);
void from_c(const pyramid_data_model_agra_EntityExternalType_c* in, pyramid::domain_model::agra::EntityExternalType& out);

void to_c(const pyramid::domain_model::agra::EntityFusionSourceType& in, pyramid_data_model_agra_EntityFusionSourceType_c* out);
void from_c(const pyramid_data_model_agra_EntityFusionSourceType_c* in, pyramid::domain_model::agra::EntityFusionSourceType& out);

void to_c(const pyramid::domain_model::agra::EntityContributorID_ChoiceType& in, pyramid_data_model_agra_EntityContributorID_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_EntityContributorID_ChoiceType_c* in, pyramid::domain_model::agra::EntityContributorID_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::EOB_EmitterID_Type& in, pyramid_data_model_agra_EOB_EmitterID_Type_c* out);
void from_c(const pyramid_data_model_agra_EOB_EmitterID_Type_c* in, pyramid::domain_model::agra::EOB_EmitterID_Type& out);

void to_c(const pyramid::domain_model::agra::SOB_C2_RecordID_Type& in, pyramid_data_model_agra_SOB_C2_RecordID_Type_c* out);
void from_c(const pyramid_data_model_agra_SOB_C2_RecordID_Type_c* in, pyramid::domain_model::agra::SOB_C2_RecordID_Type& out);

void to_c(const pyramid::domain_model::agra::SOB_SatelliteRecordID_Type& in, pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c* out);
void from_c(const pyramid_data_model_agra_SOB_SatelliteRecordID_Type_c* in, pyramid::domain_model::agra::SOB_SatelliteRecordID_Type& out);

void to_c(const pyramid::domain_model::agra::MeasurementID_Type& in, pyramid_data_model_agra_MeasurementID_Type_c* out);
void from_c(const pyramid_data_model_agra_MeasurementID_Type_c* in, pyramid::domain_model::agra::MeasurementID_Type& out);

void to_c(const pyramid::domain_model::agra::IdentityConfidenceType& in, pyramid_data_model_agra_IdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_IdentityConfidenceType_c* in, pyramid::domain_model::agra::IdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::StandardIdentityConfidenceType& in, pyramid_data_model_agra_StandardIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_StandardIdentityConfidenceType_c* in, pyramid::domain_model::agra::StandardIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::ExerciseIdentityType& in, pyramid_data_model_agra_ExerciseIdentityType_c* out);
void from_c(const pyramid_data_model_agra_ExerciseIdentityType_c* in, pyramid::domain_model::agra::ExerciseIdentityType& out);

void to_c(const pyramid::domain_model::agra::EnvironmentIdentityConfidenceType& in, pyramid_data_model_agra_EnvironmentIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_EnvironmentIdentityConfidenceType_c* in, pyramid::domain_model::agra::EnvironmentIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::PlatformIdentityConfidenceType& in, pyramid_data_model_agra_PlatformIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_PlatformIdentityConfidenceType_c* in, pyramid::domain_model::agra::PlatformIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::SpecificIdentityConfidenceType& in, pyramid_data_model_agra_SpecificIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_SpecificIdentityConfidenceType_c* in, pyramid::domain_model::agra::SpecificIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::EmitterMultipleType& in, pyramid_data_model_agra_EmitterMultipleType_c* out);
void from_c(const pyramid_data_model_agra_EmitterMultipleType_c* in, pyramid::domain_model::agra::EmitterMultipleType& out);

void to_c(const pyramid::domain_model::agra::EmitterIdentityConfidenceType& in, pyramid_data_model_agra_EmitterIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_EmitterIdentityConfidenceType_c* in, pyramid::domain_model::agra::EmitterIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::SpecificEmitterMultipleType& in, pyramid_data_model_agra_SpecificEmitterMultipleType_c* out);
void from_c(const pyramid_data_model_agra_SpecificEmitterMultipleType_c* in, pyramid::domain_model::agra::SpecificEmitterMultipleType& out);

void to_c(const pyramid::domain_model::agra::SpecificEmitterIdentityConfidenceType& in, pyramid_data_model_agra_SpecificEmitterIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_SpecificEmitterIdentityConfidenceType_c* in, pyramid::domain_model::agra::SpecificEmitterIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::SpecificVehicleIdentityConfidenceType& in, pyramid_data_model_agra_SpecificVehicleIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_SpecificVehicleIdentityConfidenceType_c* in, pyramid::domain_model::agra::SpecificVehicleIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::SpecificFacilityIdentityConfidenceType& in, pyramid_data_model_agra_SpecificFacilityIdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_SpecificFacilityIdentityConfidenceType_c* in, pyramid::domain_model::agra::SpecificFacilityIdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::EOB_IdentityConfidenceType& in, pyramid_data_model_agra_EOB_IdentityConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_EOB_IdentityConfidenceType_c* in, pyramid::domain_model::agra::EOB_IdentityConfidenceType& out);

void to_c(const pyramid::domain_model::agra::StoreMultipleType& in, pyramid_data_model_agra_StoreMultipleType_c* out);
void from_c(const pyramid_data_model_agra_StoreMultipleType_c* in, pyramid::domain_model::agra::StoreMultipleType& out);

void to_c(const pyramid::domain_model::agra::StoreConfidenceType& in, pyramid_data_model_agra_StoreConfidenceType_c* out);
void from_c(const pyramid_data_model_agra_StoreConfidenceType_c* in, pyramid::domain_model::agra::StoreConfidenceType& out);

void to_c(const pyramid::domain_model::agra::EntitySourceIdentifierType_Fusion_List& in, pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c* out);
void from_c(const pyramid_data_model_agra_EntitySourceIdentifierType_Fusion_List_c* in, pyramid::domain_model::agra::EntitySourceIdentifierType_Fusion_List& out);

void to_c(const pyramid::domain_model::agra::EntityCapabilitySourceType& in, pyramid_data_model_agra_EntityCapabilitySourceType_c* out);
void from_c(const pyramid_data_model_agra_EntityCapabilitySourceType_c* in, pyramid::domain_model::agra::EntityCapabilitySourceType& out);

void to_c(const pyramid::domain_model::agra::ProductMetadataID_Type& in, pyramid_data_model_agra_ProductMetadataID_Type_c* out);
void from_c(const pyramid_data_model_agra_ProductMetadataID_Type_c* in, pyramid::domain_model::agra::ProductMetadataID_Type& out);

void to_c(const pyramid::domain_model::agra::EntitySourceIdentifierType_ProductMetadataID_List& in, pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c* out);
void from_c(const pyramid_data_model_agra_EntitySourceIdentifierType_ProductMetadataID_List_c* in, pyramid::domain_model::agra::EntitySourceIdentifierType_ProductMetadataID_List& out);

void to_c(const pyramid::domain_model::agra::EntityIdentityType& in, pyramid_data_model_agra_EntityIdentityType_c* out);
void from_c(const pyramid_data_model_agra_EntityIdentityType_c* in, pyramid::domain_model::agra::EntityIdentityType& out);

void to_c(const pyramid::domain_model::agra::QualifyingTagsType& in, pyramid_data_model_agra_QualifyingTagsType_c* out);
void from_c(const pyramid_data_model_agra_QualifyingTagsType_c* in, pyramid::domain_model::agra::QualifyingTagsType& out);

void to_c(const pyramid::domain_model::agra::SystemMessageIdentifierType& in, pyramid_data_model_agra_SystemMessageIdentifierType_c* out);
void from_c(const pyramid_data_model_agra_SystemMessageIdentifierType_c* in, pyramid::domain_model::agra::SystemMessageIdentifierType& out);

void to_c(const pyramid::domain_model::agra::NetworkLinkID_Type& in, pyramid_data_model_agra_NetworkLinkID_Type_c* out);
void from_c(const pyramid_data_model_agra_NetworkLinkID_Type_c* in, pyramid::domain_model::agra::NetworkLinkID_Type& out);

void to_c(const pyramid::domain_model::agra::TimeFunctionType& in, pyramid_data_model_agra_TimeFunctionType_c* out);
void from_c(const pyramid_data_model_agra_TimeFunctionType_c* in, pyramid::domain_model::agra::TimeFunctionType& out);

void to_c(const pyramid::domain_model::agra::PlatformStatusType& in, pyramid_data_model_agra_PlatformStatusType_c* out);
void from_c(const pyramid_data_model_agra_PlatformStatusType_c* in, pyramid::domain_model::agra::PlatformStatusType& out);

void to_c(const pyramid::domain_model::agra::PlatformFunctionStatusType& in, pyramid_data_model_agra_PlatformFunctionStatusType_c* out);
void from_c(const pyramid_data_model_agra_PlatformFunctionStatusType_c* in, pyramid::domain_model::agra::PlatformFunctionStatusType& out);

void to_c(const pyramid::domain_model::agra::PlatformFunctionStatusCategoryType& in, pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c* out);
void from_c(const pyramid_data_model_agra_PlatformFunctionStatusCategoryType_c* in, pyramid::domain_model::agra::PlatformFunctionStatusCategoryType& out);

void to_c(const pyramid::domain_model::agra::PlatformStatusSAM_Type& in, pyramid_data_model_agra_PlatformStatusSAM_Type_c* out);
void from_c(const pyramid_data_model_agra_PlatformStatusSAM_Type_c* in, pyramid::domain_model::agra::PlatformStatusSAM_Type& out);

void to_c(const pyramid::domain_model::agra::SurfaceRecoveryType& in, pyramid_data_model_agra_SurfaceRecoveryType_c* out);
void from_c(const pyramid_data_model_agra_SurfaceRecoveryType_c* in, pyramid::domain_model::agra::SurfaceRecoveryType& out);

void to_c(const pyramid::domain_model::agra::DatalinkControlType& in, pyramid_data_model_agra_DatalinkControlType_c* out);
void from_c(const pyramid_data_model_agra_DatalinkControlType_c* in, pyramid::domain_model::agra::DatalinkControlType& out);

void to_c(const pyramid::domain_model::agra::EntitySignalSummaryType& in, pyramid_data_model_agra_EntitySignalSummaryType_c* out);
void from_c(const pyramid_data_model_agra_EntitySignalSummaryType_c* in, pyramid::domain_model::agra::EntitySignalSummaryType& out);

void to_c(const pyramid::domain_model::agra::SignalSummaryType& in, pyramid_data_model_agra_SignalSummaryType_c* out);
void from_c(const pyramid_data_model_agra_SignalSummaryType_c* in, pyramid::domain_model::agra::SignalSummaryType& out);

void to_c(const pyramid::domain_model::agra::FrequencyRangeType& in, pyramid_data_model_agra_FrequencyRangeType_c* out);
void from_c(const pyramid_data_model_agra_FrequencyRangeType_c* in, pyramid::domain_model::agra::FrequencyRangeType& out);

void to_c(const pyramid::domain_model::agra::FrequencyControlType& in, pyramid_data_model_agra_FrequencyControlType_c* out);
void from_c(const pyramid_data_model_agra_FrequencyControlType_c* in, pyramid::domain_model::agra::FrequencyControlType& out);

void to_c(const pyramid::domain_model::agra::PulseDataID_Type& in, pyramid_data_model_agra_PulseDataID_Type_c* out);
void from_c(const pyramid_data_model_agra_PulseDataID_Type_c* in, pyramid::domain_model::agra::PulseDataID_Type& out);

void to_c(const pyramid::domain_model::agra::StrengthType& in, pyramid_data_model_agra_StrengthType_c* out);
void from_c(const pyramid_data_model_agra_StrengthType_c* in, pyramid::domain_model::agra::StrengthType& out);

void to_c(const pyramid::domain_model::agra::StrengthRangeType& in, pyramid_data_model_agra_StrengthRangeType_c* out);
void from_c(const pyramid_data_model_agra_StrengthRangeType_c* in, pyramid::domain_model::agra::StrengthRangeType& out);

void to_c(const pyramid::domain_model::agra::ActivityAgainstType& in, pyramid_data_model_agra_ActivityAgainstType_c* out);
void from_c(const pyramid_data_model_agra_ActivityAgainstType_c* in, pyramid::domain_model::agra::ActivityAgainstType& out);

void to_c(const pyramid::domain_model::agra::ActivityActorID_ChoiceType& in, pyramid_data_model_agra_ActivityActorID_ChoiceType_c* out);
void from_c(const pyramid_data_model_agra_ActivityActorID_ChoiceType_c* in, pyramid::domain_model::agra::ActivityActorID_ChoiceType& out);

void to_c(const pyramid::domain_model::agra::VoiceControlType& in, pyramid_data_model_agra_VoiceControlType_c* out);
void from_c(const pyramid_data_model_agra_VoiceControlType_c* in, pyramid::domain_model::agra::VoiceControlType& out);

void to_c(const pyramid::domain_model::agra::EntityRemoveInfoType& in, pyramid_data_model_agra_EntityRemoveInfoType_c* out);
void from_c(const pyramid_data_model_agra_EntityRemoveInfoType_c* in, pyramid::domain_model::agra::EntityRemoveInfoType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSingleVectorParametersType& in, pyramid_data_model_agra_OrbitalSingleVectorParametersType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSingleVectorParametersType_c* in, pyramid::domain_model::agra::OrbitalSingleVectorParametersType& out);

void to_c(const pyramid::domain_model::agra::EphemerisOrbitalModelType& in, pyramid_data_model_agra_EphemerisOrbitalModelType_c* out);
void from_c(const pyramid_data_model_agra_EphemerisOrbitalModelType_c* in, pyramid::domain_model::agra::EphemerisOrbitalModelType& out);

void to_c(const pyramid::domain_model::agra::OrbitalModelType& in, pyramid_data_model_agra_OrbitalModelType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalModelType_c* in, pyramid::domain_model::agra::OrbitalModelType& out);

void to_c(const pyramid::domain_model::agra::EntityMetadataMDT& in, pyramid_data_model_agra_EntityMetadataMDT_c* out);
void from_c(const pyramid_data_model_agra_EntityMetadataMDT_c* in, pyramid::domain_model::agra::EntityMetadataMDT& out);

void to_c(const pyramid::domain_model::agra::MetadataID_Type& in, pyramid_data_model_agra_MetadataID_Type_c* out);
void from_c(const pyramid_data_model_agra_MetadataID_Type_c* in, pyramid::domain_model::agra::MetadataID_Type& out);

void to_c(const pyramid::domain_model::agra::EntityMetadataPET& in, pyramid_data_model_agra_EntityMetadataPET_c* out);
void from_c(const pyramid_data_model_agra_EntityMetadataPET_c* in, pyramid::domain_model::agra::EntityMetadataPET& out);

void to_c(const pyramid::domain_model::agra::SystemDataType& in, pyramid_data_model_agra_SystemDataType_c* out);
void from_c(const pyramid_data_model_agra_SystemDataType_c* in, pyramid::domain_model::agra::SystemDataType& out);

void to_c(const pyramid::domain_model::agra::SystemStatusMDT& in, pyramid_data_model_agra_SystemStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_SystemStatusMDT_c* in, pyramid::domain_model::agra::SystemStatusMDT& out);

void to_c(const pyramid::domain_model::agra::SystemIdentityType& in, pyramid_data_model_agra_SystemIdentityType_c* out);
void from_c(const pyramid_data_model_agra_SystemIdentityType_c* in, pyramid::domain_model::agra::SystemIdentityType& out);

void to_c(const pyramid::domain_model::agra::SystemCommunicationsType& in, pyramid_data_model_agra_SystemCommunicationsType_c* out);
void from_c(const pyramid_data_model_agra_SystemCommunicationsType_c* in, pyramid::domain_model::agra::SystemCommunicationsType& out);

void to_c(const pyramid::domain_model::agra::CommSystemUsageType& in, pyramid_data_model_agra_CommSystemUsageType_c* out);
void from_c(const pyramid_data_model_agra_CommSystemUsageType_c* in, pyramid::domain_model::agra::CommSystemUsageType& out);

void to_c(const pyramid::domain_model::agra::CommSystemID_Type& in, pyramid_data_model_agra_CommSystemID_Type_c* out);
void from_c(const pyramid_data_model_agra_CommSystemID_Type_c* in, pyramid::domain_model::agra::CommSystemID_Type& out);

void to_c(const pyramid::domain_model::agra::CommModeUsageType& in, pyramid_data_model_agra_CommModeUsageType_c* out);
void from_c(const pyramid_data_model_agra_CommModeUsageType_c* in, pyramid::domain_model::agra::CommModeUsageType& out);

void to_c(const pyramid::domain_model::agra::CommAntennaModeType& in, pyramid_data_model_agra_CommAntennaModeType_c* out);
void from_c(const pyramid_data_model_agra_CommAntennaModeType_c* in, pyramid::domain_model::agra::CommAntennaModeType& out);

void to_c(const pyramid::domain_model::agra::SubsystemID_Type& in, pyramid_data_model_agra_SubsystemID_Type_c* out);
void from_c(const pyramid_data_model_agra_SubsystemID_Type_c* in, pyramid::domain_model::agra::SubsystemID_Type& out);

void to_c(const pyramid::domain_model::agra::PositionReportMDT& in, pyramid_data_model_agra_PositionReportMDT_c* out);
void from_c(const pyramid_data_model_agra_PositionReportMDT_c* in, pyramid::domain_model::agra::PositionReportMDT& out);

void to_c(const pyramid::domain_model::agra::InertialStateType& in, pyramid_data_model_agra_InertialStateType_c* out);
void from_c(const pyramid_data_model_agra_InertialStateType_c* in, pyramid::domain_model::agra::InertialStateType& out);

void to_c(const pyramid::domain_model::agra::Point4D_Type& in, pyramid_data_model_agra_Point4D_Type_c* out);
void from_c(const pyramid_data_model_agra_Point4D_Type_c* in, pyramid::domain_model::agra::Point4D_Type& out);

void to_c(const pyramid::domain_model::agra::Velocity3D_Type& in, pyramid_data_model_agra_Velocity3D_Type_c* out);
void from_c(const pyramid_data_model_agra_Velocity3D_Type_c* in, pyramid::domain_model::agra::Velocity3D_Type& out);

void to_c(const pyramid::domain_model::agra::OrientationRateType& in, pyramid_data_model_agra_OrientationRateType_c* out);
void from_c(const pyramid_data_model_agra_OrientationRateType_c* in, pyramid::domain_model::agra::OrientationRateType& out);

void to_c(const pyramid::domain_model::agra::NavigationReportMDT& in, pyramid_data_model_agra_NavigationReportMDT_c* out);
void from_c(const pyramid_data_model_agra_NavigationReportMDT_c* in, pyramid::domain_model::agra::NavigationReportMDT& out);

void to_c(const pyramid::domain_model::agra::NavigationType& in, pyramid_data_model_agra_NavigationType_c* out);
void from_c(const pyramid_data_model_agra_NavigationType_c* in, pyramid::domain_model::agra::NavigationType& out);

void to_c(const pyramid::domain_model::agra::NavigationSourceType& in, pyramid_data_model_agra_NavigationSourceType_c* out);
void from_c(const pyramid_data_model_agra_NavigationSourceType_c* in, pyramid::domain_model::agra::NavigationSourceType& out);

void to_c(const pyramid::domain_model::agra::MissionPlanNavigationType& in, pyramid_data_model_agra_MissionPlanNavigationType_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanNavigationType_c* in, pyramid::domain_model::agra::MissionPlanNavigationType& out);

void to_c(const pyramid::domain_model::agra::Point3D_Type& in, pyramid_data_model_agra_Point3D_Type_c* out);
void from_c(const pyramid_data_model_agra_Point3D_Type_c* in, pyramid::domain_model::agra::Point3D_Type& out);

void to_c(const pyramid::domain_model::agra::RelativeNavigationType& in, pyramid_data_model_agra_RelativeNavigationType_c* out);
void from_c(const pyramid_data_model_agra_RelativeNavigationType_c* in, pyramid::domain_model::agra::RelativeNavigationType& out);

void to_c(const pyramid::domain_model::agra::SlavedNavigationType& in, pyramid_data_model_agra_SlavedNavigationType_c* out);
void from_c(const pyramid_data_model_agra_SlavedNavigationType_c* in, pyramid::domain_model::agra::SlavedNavigationType& out);

void to_c(const pyramid::domain_model::agra::SystemMetadataMDT& in, pyramid_data_model_agra_SystemMetadataMDT_c* out);
void from_c(const pyramid_data_model_agra_SystemMetadataMDT_c* in, pyramid::domain_model::agra::SystemMetadataMDT& out);

void to_c(const pyramid::domain_model::agra::SystemMetadataPET& in, pyramid_data_model_agra_SystemMetadataPET_c* out);
void from_c(const pyramid_data_model_agra_SystemMetadataPET_c* in, pyramid::domain_model::agra::SystemMetadataPET& out);

void to_c(const pyramid::domain_model::agra::EmergencyReferenceOpPointType& in, pyramid_data_model_agra_EmergencyReferenceOpPointType_c* out);
void from_c(const pyramid_data_model_agra_EmergencyReferenceOpPointType_c* in, pyramid::domain_model::agra::EmergencyReferenceOpPointType& out);

void to_c(const pyramid::domain_model::agra::OpPointBaseType& in, pyramid_data_model_agra_OpPointBaseType_c* out);
void from_c(const pyramid_data_model_agra_OpPointBaseType_c* in, pyramid::domain_model::agra::OpPointBaseType& out);

void to_c(const pyramid::domain_model::agra::OpBaseType& in, pyramid_data_model_agra_OpBaseType_c* out);
void from_c(const pyramid_data_model_agra_OpBaseType_c* in, pyramid::domain_model::agra::OpBaseType& out);

void to_c(const pyramid::domain_model::agra::OpDescriptionType& in, pyramid_data_model_agra_OpDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_OpDescriptionType_c* in, pyramid::domain_model::agra::OpDescriptionType& out);

void to_c(const pyramid::domain_model::agra::SystemScheduleStateType& in, pyramid_data_model_agra_SystemScheduleStateType_c* out);
void from_c(const pyramid_data_model_agra_SystemScheduleStateType_c* in, pyramid::domain_model::agra::SystemScheduleStateType& out);

void to_c(const pyramid::domain_model::agra::ScheduleStateType& in, pyramid_data_model_agra_ScheduleStateType_c* out);
void from_c(const pyramid_data_model_agra_ScheduleStateType_c* in, pyramid::domain_model::agra::ScheduleStateType& out);

void to_c(const pyramid::domain_model::agra::OpPointChoiceType& in, pyramid_data_model_agra_OpPointChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OpPointChoiceType_c* in, pyramid::domain_model::agra::OpPointChoiceType& out);

void to_c(const pyramid::domain_model::agra::OpPointPositionType& in, pyramid_data_model_agra_OpPointPositionType_c* out);
void from_c(const pyramid_data_model_agra_OpPointPositionType_c* in, pyramid::domain_model::agra::OpPointPositionType& out);

void to_c(const pyramid::domain_model::agra::KinematicsFixedPositionType& in, pyramid_data_model_agra_KinematicsFixedPositionType_c* out);
void from_c(const pyramid_data_model_agra_KinematicsFixedPositionType_c* in, pyramid::domain_model::agra::KinematicsFixedPositionType& out);

void to_c(const pyramid::domain_model::agra::SystemConfigurationType& in, pyramid_data_model_agra_SystemConfigurationType_c* out);
void from_c(const pyramid_data_model_agra_SystemConfigurationType_c* in, pyramid::domain_model::agra::SystemConfigurationType& out);

void to_c(const pyramid::domain_model::agra::VehicleConfigurationType& in, pyramid_data_model_agra_VehicleConfigurationType_c* out);
void from_c(const pyramid_data_model_agra_VehicleConfigurationType_c* in, pyramid::domain_model::agra::VehicleConfigurationType& out);

void to_c(const pyramid::domain_model::agra::InertialStateRelativeType& in, pyramid_data_model_agra_InertialStateRelativeType_c* out);
void from_c(const pyramid_data_model_agra_InertialStateRelativeType_c* in, pyramid::domain_model::agra::InertialStateRelativeType& out);

void to_c(const pyramid::domain_model::agra::PointChoice4D_Type& in, pyramid_data_model_agra_PointChoice4D_Type_c* out);
void from_c(const pyramid_data_model_agra_PointChoice4D_Type_c* in, pyramid::domain_model::agra::PointChoice4D_Type& out);

void to_c(const pyramid::domain_model::agra::Point4D_RelativeType& in, pyramid_data_model_agra_Point4D_RelativeType_c* out);
void from_c(const pyramid_data_model_agra_Point4D_RelativeType_c* in, pyramid::domain_model::agra::Point4D_RelativeType& out);

void to_c(const pyramid::domain_model::agra::RelativeOffset3D_Type& in, pyramid_data_model_agra_RelativeOffset3D_Type_c* out);
void from_c(const pyramid_data_model_agra_RelativeOffset3D_Type_c* in, pyramid::domain_model::agra::RelativeOffset3D_Type& out);

void to_c(const pyramid::domain_model::agra::SafeAltitudeType& in, pyramid_data_model_agra_SafeAltitudeType_c* out);
void from_c(const pyramid_data_model_agra_SafeAltitudeType_c* in, pyramid::domain_model::agra::SafeAltitudeType& out);

void to_c(const pyramid::domain_model::agra::AltitudeRadialPairType& in, pyramid_data_model_agra_AltitudeRadialPairType_c* out);
void from_c(const pyramid_data_model_agra_AltitudeRadialPairType_c* in, pyramid::domain_model::agra::AltitudeRadialPairType& out);

void to_c(const pyramid::domain_model::agra::EmergencyReferenceOpPointCategoriesType& in, pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c* out);
void from_c(const pyramid_data_model_agra_EmergencyReferenceOpPointCategoriesType_c* in, pyramid::domain_model::agra::EmergencyReferenceOpPointCategoriesType& out);

void to_c(const pyramid::domain_model::agra::OpLineMDT& in, pyramid_data_model_agra_OpLineMDT_c* out);
void from_c(const pyramid_data_model_agra_OpLineMDT_c* in, pyramid::domain_model::agra::OpLineMDT& out);

void to_c(const pyramid::domain_model::agra::OpLineType& in, pyramid_data_model_agra_OpLineType_c* out);
void from_c(const pyramid_data_model_agra_OpLineType_c* in, pyramid::domain_model::agra::OpLineType& out);

void to_c(const pyramid::domain_model::agra::OpZoneMDT& in, pyramid_data_model_agra_OpZoneMDT_c* out);
void from_c(const pyramid_data_model_agra_OpZoneMDT_c* in, pyramid::domain_model::agra::OpZoneMDT& out);

void to_c(const pyramid::domain_model::agra::OpZoneCategoryType& in, pyramid_data_model_agra_OpZoneCategoryType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneCategoryType_c* in, pyramid::domain_model::agra::OpZoneCategoryType& out);

void to_c(const pyramid::domain_model::agra::ConstrainedEntryExitType& in, pyramid_data_model_agra_ConstrainedEntryExitType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainedEntryExitType_c* in, pyramid::domain_model::agra::ConstrainedEntryExitType& out);

void to_c(const pyramid::domain_model::agra::OpZoneFilterAreaPET& in, pyramid_data_model_agra_OpZoneFilterAreaPET_c* out);
void from_c(const pyramid_data_model_agra_OpZoneFilterAreaPET_c* in, pyramid::domain_model::agra::OpZoneFilterAreaPET& out);

void to_c(const pyramid::domain_model::agra::OpZoneCategoryType_FilterArea_List& in, pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c* out);
void from_c(const pyramid_data_model_agra_OpZoneCategoryType_FilterArea_List_c* in, pyramid::domain_model::agra::OpZoneCategoryType_FilterArea_List& out);

void to_c(const pyramid::domain_model::agra::OpZoneJammingType& in, pyramid_data_model_agra_OpZoneJammingType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneJammingType_c* in, pyramid::domain_model::agra::OpZoneJammingType& out);

void to_c(const pyramid::domain_model::agra::IngressEgressType& in, pyramid_data_model_agra_IngressEgressType_c* out);
void from_c(const pyramid_data_model_agra_IngressEgressType_c* in, pyramid::domain_model::agra::IngressEgressType& out);

void to_c(const pyramid::domain_model::agra::OpZoneMissileDataType& in, pyramid_data_model_agra_OpZoneMissileDataType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneMissileDataType_c* in, pyramid::domain_model::agra::OpZoneMissileDataType& out);

void to_c(const pyramid::domain_model::agra::TrackNumberOrEntityType& in, pyramid_data_model_agra_TrackNumberOrEntityType_c* out);
void from_c(const pyramid_data_model_agra_TrackNumberOrEntityType_c* in, pyramid::domain_model::agra::TrackNumberOrEntityType& out);

void to_c(const pyramid::domain_model::agra::Link16TrackIdentifierType& in, pyramid_data_model_agra_Link16TrackIdentifierType_c* out);
void from_c(const pyramid_data_model_agra_Link16TrackIdentifierType_c* in, pyramid::domain_model::agra::Link16TrackIdentifierType& out);

void to_c(const pyramid::domain_model::agra::OpZoneNoFireType& in, pyramid_data_model_agra_OpZoneNoFireType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneNoFireType_c* in, pyramid::domain_model::agra::OpZoneNoFireType& out);

void to_c(const pyramid::domain_model::agra::OpZoneNoFlyType& in, pyramid_data_model_agra_OpZoneNoFlyType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneNoFlyType_c* in, pyramid::domain_model::agra::OpZoneNoFlyType& out);

void to_c(const pyramid::domain_model::agra::VehicleCommandDataType& in, pyramid_data_model_agra_VehicleCommandDataType_c* out);
void from_c(const pyramid_data_model_agra_VehicleCommandDataType_c* in, pyramid::domain_model::agra::VehicleCommandDataType& out);

void to_c(const pyramid::domain_model::agra::CommAllocationActionType& in, pyramid_data_model_agra_CommAllocationActionType_c* out);
void from_c(const pyramid_data_model_agra_CommAllocationActionType_c* in, pyramid::domain_model::agra::CommAllocationActionType& out);

void to_c(const pyramid::domain_model::agra::OpZoneWeaponRestrictionType& in, pyramid_data_model_agra_OpZoneWeaponRestrictionType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneWeaponRestrictionType_c* in, pyramid::domain_model::agra::OpZoneWeaponRestrictionType& out);

void to_c(const pyramid::domain_model::agra::WeaponRestrictionType& in, pyramid_data_model_agra_WeaponRestrictionType_c* out);
void from_c(const pyramid_data_model_agra_WeaponRestrictionType_c* in, pyramid::domain_model::agra::WeaponRestrictionType& out);

void to_c(const pyramid::domain_model::agra::WeaponRestrictionType_WeaponsAllowed_List& in, pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c* out);
void from_c(const pyramid_data_model_agra_WeaponRestrictionType_WeaponsAllowed_List_c* in, pyramid::domain_model::agra::WeaponRestrictionType_WeaponsAllowed_List& out);

void to_c(const pyramid::domain_model::agra::WeaponRestrictionType_WeaponsNotAllowed_List& in, pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c* out);
void from_c(const pyramid_data_model_agra_WeaponRestrictionType_WeaponsNotAllowed_List_c* in, pyramid::domain_model::agra::WeaponRestrictionType_WeaponsNotAllowed_List& out);

void to_c(const pyramid::domain_model::agra::OpZoneWeatherType& in, pyramid_data_model_agra_OpZoneWeatherType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneWeatherType_c* in, pyramid::domain_model::agra::OpZoneWeatherType& out);

void to_c(const pyramid::domain_model::agra::WeatherAreaDataType& in, pyramid_data_model_agra_WeatherAreaDataType_c* out);
void from_c(const pyramid_data_model_agra_WeatherAreaDataType_c* in, pyramid::domain_model::agra::WeatherAreaDataType& out);

void to_c(const pyramid::domain_model::agra::CloudsType& in, pyramid_data_model_agra_CloudsType_c* out);
void from_c(const pyramid_data_model_agra_CloudsType_c* in, pyramid::domain_model::agra::CloudsType& out);

void to_c(const pyramid::domain_model::agra::WeatherEffectsType& in, pyramid_data_model_agra_WeatherEffectsType_c* out);
void from_c(const pyramid_data_model_agra_WeatherEffectsType_c* in, pyramid::domain_model::agra::WeatherEffectsType& out);

void to_c(const pyramid::domain_model::agra::WindDataType& in, pyramid_data_model_agra_WindDataType_c* out);
void from_c(const pyramid_data_model_agra_WindDataType_c* in, pyramid::domain_model::agra::WindDataType& out);

void to_c(const pyramid::domain_model::agra::WindDataChoiceType& in, pyramid_data_model_agra_WindDataChoiceType_c* out);
void from_c(const pyramid_data_model_agra_WindDataChoiceType_c* in, pyramid::domain_model::agra::WindDataChoiceType& out);

void to_c(const pyramid::domain_model::agra::WindMagnitudeType& in, pyramid_data_model_agra_WindMagnitudeType_c* out);
void from_c(const pyramid_data_model_agra_WindMagnitudeType_c* in, pyramid::domain_model::agra::WindMagnitudeType& out);

void to_c(const pyramid::domain_model::agra::OpZoneType& in, pyramid_data_model_agra_OpZoneType_c* out);
void from_c(const pyramid_data_model_agra_OpZoneType_c* in, pyramid::domain_model::agra::OpZoneType& out);

void to_c(const pyramid::domain_model::agra::OpVolumeMDT& in, pyramid_data_model_agra_OpVolumeMDT_c* out);
void from_c(const pyramid_data_model_agra_OpVolumeMDT_c* in, pyramid::domain_model::agra::OpVolumeMDT& out);

void to_c(const pyramid::domain_model::agra::GeoLocatedObjectType& in, pyramid_data_model_agra_GeoLocatedObjectType_c* out);
void from_c(const pyramid_data_model_agra_GeoLocatedObjectType_c* in, pyramid::domain_model::agra::GeoLocatedObjectType& out);

void to_c(const pyramid::domain_model::agra::SignalReportID_Type& in, pyramid_data_model_agra_SignalReportID_Type_c* out);
void from_c(const pyramid_data_model_agra_SignalReportID_Type_c* in, pyramid::domain_model::agra::SignalReportID_Type& out);

void to_c(const pyramid::domain_model::agra::Link16HazardType& in, pyramid_data_model_agra_Link16HazardType_c* out);
void from_c(const pyramid_data_model_agra_Link16HazardType_c* in, pyramid::domain_model::agra::Link16HazardType& out);

void to_c(const pyramid::domain_model::agra::ConstrainedOpLineType& in, pyramid_data_model_agra_ConstrainedOpLineType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainedOpLineType_c* in, pyramid::domain_model::agra::ConstrainedOpLineType& out);

void to_c(const pyramid::domain_model::agra::ConstrainedOpZoneType& in, pyramid_data_model_agra_ConstrainedOpZoneType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainedOpZoneType_c* in, pyramid::domain_model::agra::ConstrainedOpZoneType& out);

void to_c(const pyramid::domain_model::agra::ConstrainedOpVolumeType& in, pyramid_data_model_agra_ConstrainedOpVolumeType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainedOpVolumeType_c* in, pyramid::domain_model::agra::ConstrainedOpVolumeType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementRiskAdjustmentType& in, pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementRiskAdjustmentType_c* in, pyramid::domain_model::agra::MA_RequirementRiskAdjustmentType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementChoiceType& in, pyramid_data_model_agra_MA_RequirementChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementChoiceType_c* in, pyramid::domain_model::agra::MA_RequirementChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementTaxonomyChoiceType& in, pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementTaxonomyChoiceType_c* in, pyramid::domain_model::agra::MA_RequirementTaxonomyChoiceType& out);

void to_c(const pyramid::domain_model::agra::ParameterAssertType& in, pyramid_data_model_agra_ParameterAssertType_c* out);
void from_c(const pyramid_data_model_agra_ParameterAssertType_c* in, pyramid::domain_model::agra::ParameterAssertType& out);

void to_c(const pyramid::domain_model::agra::ParameterID_Type& in, pyramid_data_model_agra_ParameterID_Type_c* out);
void from_c(const pyramid_data_model_agra_ParameterID_Type_c* in, pyramid::domain_model::agra::ParameterID_Type& out);

void to_c(const pyramid::domain_model::agra::ParameterValueType& in, pyramid_data_model_agra_ParameterValueType_c* out);
void from_c(const pyramid_data_model_agra_ParameterValueType_c* in, pyramid::domain_model::agra::ParameterValueType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementPlanningCandidateType& in, pyramid_data_model_agra_MA_RequirementPlanningCandidateType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementPlanningCandidateType_c* in, pyramid::domain_model::agra::MA_RequirementPlanningCandidateType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningCandidateBaseType& in, pyramid_data_model_agra_MA_PlanningCandidateBaseType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningCandidateBaseType_c* in, pyramid::domain_model::agra::MA_PlanningCandidateBaseType& out);

void to_c(const pyramid::domain_model::agra::MA_ConstrainingPlansType& in, pyramid_data_model_agra_MA_ConstrainingPlansType_c* out);
void from_c(const pyramid_data_model_agra_MA_ConstrainingPlansType_c* in, pyramid::domain_model::agra::MA_ConstrainingPlansType& out);

void to_c(const pyramid::domain_model::agra::MA_TaskPlanConstraintType& in, pyramid_data_model_agra_MA_TaskPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskPlanConstraintType_c* in, pyramid::domain_model::agra::MA_TaskPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanConstraintType& in, pyramid_data_model_agra_RoutePlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanConstraintType_c* in, pyramid::domain_model::agra::RoutePlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::RouteActivityPlanConstraintType& in, pyramid_data_model_agra_RouteActivityPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_RouteActivityPlanConstraintType_c* in, pyramid::domain_model::agra::RouteActivityPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanConstraintType& in, pyramid_data_model_agra_OrbitPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanConstraintType_c* in, pyramid::domain_model::agra::OrbitPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::OrbitActivityPlanConstraintType& in, pyramid_data_model_agra_OrbitActivityPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_OrbitActivityPlanConstraintType_c* in, pyramid::domain_model::agra::OrbitActivityPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::ActivityPlanConstraintType& in, pyramid_data_model_agra_ActivityPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_ActivityPlanConstraintType_c* in, pyramid::domain_model::agra::ActivityPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::CommAllocationConstraintType& in, pyramid_data_model_agra_CommAllocationConstraintType_c* out);
void from_c(const pyramid_data_model_agra_CommAllocationConstraintType_c* in, pyramid::domain_model::agra::CommAllocationConstraintType& out);

void to_c(const pyramid::domain_model::agra::EffectPlanConstraintType& in, pyramid_data_model_agra_EffectPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_EffectPlanConstraintType_c* in, pyramid::domain_model::agra::EffectPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::ActionPlanConstraintType& in, pyramid_data_model_agra_ActionPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_ActionPlanConstraintType_c* in, pyramid::domain_model::agra::ActionPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::ResponsePlanConstraintType& in, pyramid_data_model_agra_ResponsePlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_ResponsePlanConstraintType_c* in, pyramid::domain_model::agra::ResponsePlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::MA_OtherSystemConstrainingPlansType& in, pyramid_data_model_agra_MA_OtherSystemConstrainingPlansType_c* out);
void from_c(const pyramid_data_model_agra_MA_OtherSystemConstrainingPlansType_c* in, pyramid::domain_model::agra::MA_OtherSystemConstrainingPlansType& out);

void to_c(const pyramid::domain_model::agra::PlanningGuidelineType& in, pyramid_data_model_agra_PlanningGuidelineType_c* out);
void from_c(const pyramid_data_model_agra_PlanningGuidelineType_c* in, pyramid::domain_model::agra::PlanningGuidelineType& out);

void to_c(const pyramid::domain_model::agra::PlanningPointType& in, pyramid_data_model_agra_PlanningPointType_c* out);
void from_c(const pyramid_data_model_agra_PlanningPointType_c* in, pyramid::domain_model::agra::PlanningPointType& out);

void to_c(const pyramid::domain_model::agra::PlanningLocationType& in, pyramid_data_model_agra_PlanningLocationType_c* out);
void from_c(const pyramid_data_model_agra_PlanningLocationType_c* in, pyramid::domain_model::agra::PlanningLocationType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanReferenceType& in, pyramid_data_model_agra_RoutePlanReferenceType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanReferenceType_c* in, pyramid::domain_model::agra::RoutePlanReferenceType& out);

void to_c(const pyramid::domain_model::agra::PlanningPointPriorityType& in, pyramid_data_model_agra_PlanningPointPriorityType_c* out);
void from_c(const pyramid_data_model_agra_PlanningPointPriorityType_c* in, pyramid::domain_model::agra::PlanningPointPriorityType& out);

void to_c(const pyramid::domain_model::agra::OrbitGuidelineType& in, pyramid_data_model_agra_OrbitGuidelineType_c* out);
void from_c(const pyramid_data_model_agra_OrbitGuidelineType_c* in, pyramid::domain_model::agra::OrbitGuidelineType& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanningStateType& in, pyramid_data_model_agra_OrbitPlanningStateType_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanningStateType_c* in, pyramid::domain_model::agra::OrbitPlanningStateType& out);

void to_c(const pyramid::domain_model::agra::OrbitTransitionSequenceType& in, pyramid_data_model_agra_OrbitTransitionSequenceType_c* out);
void from_c(const pyramid_data_model_agra_OrbitTransitionSequenceType_c* in, pyramid::domain_model::agra::OrbitTransitionSequenceType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementPlanConstraintType& in, pyramid_data_model_agra_MA_RequirementPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementPlanConstraintType_c* in, pyramid::domain_model::agra::MA_RequirementPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementAllocationCommandType& in, pyramid_data_model_agra_MA_RequirementAllocationCommandType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementAllocationCommandType_c* in, pyramid::domain_model::agra::MA_RequirementAllocationCommandType& out);

void to_c(const pyramid::domain_model::agra::MA_EffectAllocationType& in, pyramid_data_model_agra_MA_EffectAllocationType_c* out);
void from_c(const pyramid_data_model_agra_MA_EffectAllocationType_c* in, pyramid::domain_model::agra::MA_EffectAllocationType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementAllocationBaseType& in, pyramid_data_model_agra_MA_RequirementAllocationBaseType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementAllocationBaseType_c* in, pyramid::domain_model::agra::MA_RequirementAllocationBaseType& out);

void to_c(const pyramid::domain_model::agra::RequirementAllocationDetailsType& in, pyramid_data_model_agra_RequirementAllocationDetailsType_c* out);
void from_c(const pyramid_data_model_agra_RequirementAllocationDetailsType_c* in, pyramid::domain_model::agra::RequirementAllocationDetailsType& out);

void to_c(const pyramid::domain_model::agra::DMPI_AllocationType& in, pyramid_data_model_agra_DMPI_AllocationType_c* out);
void from_c(const pyramid_data_model_agra_DMPI_AllocationType_c* in, pyramid::domain_model::agra::DMPI_AllocationType& out);

void to_c(const pyramid::domain_model::agra::AO_CodeType& in, pyramid_data_model_agra_AO_CodeType_c* out);
void from_c(const pyramid_data_model_agra_AO_CodeType_c* in, pyramid::domain_model::agra::AO_CodeType& out);

void to_c(const pyramid::domain_model::agra::MA_ActionAllocationType& in, pyramid_data_model_agra_MA_ActionAllocationType_c* out);
void from_c(const pyramid_data_model_agra_MA_ActionAllocationType_c* in, pyramid::domain_model::agra::MA_ActionAllocationType& out);

void to_c(const pyramid::domain_model::agra::MA_TaskAllocationType& in, pyramid_data_model_agra_MA_TaskAllocationType_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskAllocationType_c* in, pyramid::domain_model::agra::MA_TaskAllocationType& out);

void to_c(const pyramid::domain_model::agra::MA_ResponseAllocationType& in, pyramid_data_model_agra_MA_ResponseAllocationType_c* out);
void from_c(const pyramid_data_model_agra_MA_ResponseAllocationType_c* in, pyramid::domain_model::agra::MA_ResponseAllocationType& out);

void to_c(const pyramid::domain_model::agra::RequirementAssociationConstraintType& in, pyramid_data_model_agra_RequirementAssociationConstraintType_c* out);
void from_c(const pyramid_data_model_agra_RequirementAssociationConstraintType_c* in, pyramid::domain_model::agra::RequirementAssociationConstraintType& out);

void to_c(const pyramid::domain_model::agra::AssociatedRequirementsType& in, pyramid_data_model_agra_AssociatedRequirementsType_c* out);
void from_c(const pyramid_data_model_agra_AssociatedRequirementsType_c* in, pyramid::domain_model::agra::AssociatedRequirementsType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationCommandMT& in, pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationCommandMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationCommandMDT& in, pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationCommandMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationCommandMDT& out);

void to_c(const pyramid::domain_model::agra::MissionPlanActivationCommandID_Type& in, pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c* out);
void from_c(const pyramid_data_model_agra_MissionPlanActivationCommandID_Type_c* in, pyramid::domain_model::agra::MissionPlanActivationCommandID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMT& in, pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMDT& in, pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMDT& out);

void to_c(const pyramid::domain_model::agra::PlanCommandStatusType& in, pyramid_data_model_agra_PlanCommandStatusType_c* out);
void from_c(const pyramid_data_model_agra_PlanCommandStatusType_c* in, pyramid::domain_model::agra::PlanCommandStatusType& out);

void to_c(const pyramid::domain_model::agra::CompletionStatusType& in, pyramid_data_model_agra_CompletionStatusType_c* out);
void from_c(const pyramid_data_model_agra_CompletionStatusType_c* in, pyramid::domain_model::agra::CompletionStatusType& out);

void to_c(const pyramid::domain_model::agra::IncompleteProcessingType& in, pyramid_data_model_agra_IncompleteProcessingType_c* out);
void from_c(const pyramid_data_model_agra_IncompleteProcessingType_c* in, pyramid::domain_model::agra::IncompleteProcessingType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanActivationStatusType& in, pyramid_data_model_agra_MA_PlanActivationStatusType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanActivationStatusType_c* in, pyramid::domain_model::agra::MA_PlanActivationStatusType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationStatusMT& in, pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanActivationStatusMDT& in, pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanActivationStatusMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanActivationStatusMDT& out);

void to_c(const pyramid::domain_model::agra::MA_PlanActivationStateType& in, pyramid_data_model_agra_MA_PlanActivationStateType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanActivationStateType_c* in, pyramid::domain_model::agra::MA_PlanActivationStateType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanCommandMT& in, pyramid_data_model_agra_MA_MissionPlanCommandMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanCommandMT_c* in, pyramid::domain_model::agra::MA_MissionPlanCommandMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanCommandMDT& in, pyramid_data_model_agra_MA_MissionPlanCommandMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanCommandMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanCommandMDT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanCommandStatusMT& in, pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c* in, pyramid::domain_model::agra::MA_MissionPlanCommandStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanCommandStatusMDT& in, pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanCommandStatusMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanCommandStatusMDT& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementPlanningResultType& in, pyramid_data_model_agra_MA_RequirementPlanningResultType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementPlanningResultType_c* in, pyramid::domain_model::agra::MA_RequirementPlanningResultType& out);

void to_c(const pyramid::domain_model::agra::MA_RequirementPlanningResultBaseType& in, pyramid_data_model_agra_MA_RequirementPlanningResultBaseType_c* out);
void from_c(const pyramid_data_model_agra_MA_RequirementPlanningResultBaseType_c* in, pyramid::domain_model::agra::MA_RequirementPlanningResultBaseType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanReferenceType& in, pyramid_data_model_agra_MA_PlanReferenceType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanReferenceType_c* in, pyramid::domain_model::agra::MA_PlanReferenceType& out);

void to_c(const pyramid::domain_model::agra::UnallocatedReasonType& in, pyramid_data_model_agra_UnallocatedReasonType_c* out);
void from_c(const pyramid_data_model_agra_UnallocatedReasonType_c* in, pyramid::domain_model::agra::UnallocatedReasonType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanExecutionStatusMT& in, pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c* in, pyramid::domain_model::agra::MA_MissionPlanExecutionStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanExecutionStatusMDT& in, pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanExecutionStatusMDT_c* in, pyramid::domain_model::agra::MA_MissionPlanExecutionStatusMDT& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanExecutionStateType& in, pyramid_data_model_agra_MA_MissionPlanExecutionStateType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanExecutionStateType_c* in, pyramid::domain_model::agra::MA_MissionPlanExecutionStateType& out);

void to_c(const pyramid::domain_model::agra::MA_ExecutionPlanSetExecutionStateType& in, pyramid_data_model_agra_MA_ExecutionPlanSetExecutionStateType_c* out);
void from_c(const pyramid_data_model_agra_MA_ExecutionPlanSetExecutionStateType_c* in, pyramid::domain_model::agra::MA_ExecutionPlanSetExecutionStateType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionMT& in, pyramid_data_model_agra_MA_PlanningFunctionMT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionMT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionMT& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionMDT& in, pyramid_data_model_agra_MA_PlanningFunctionMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionMDT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionMDT& out);

void to_c(const pyramid::domain_model::agra::PlanningFunctionID_Type& in, pyramid_data_model_agra_PlanningFunctionID_Type_c* out);
void from_c(const pyramid_data_model_agra_PlanningFunctionID_Type_c* in, pyramid::domain_model::agra::PlanningFunctionID_Type& out);

void to_c(const pyramid::domain_model::agra::PlanningInterfacesType& in, pyramid_data_model_agra_PlanningInterfacesType_c* out);
void from_c(const pyramid_data_model_agra_PlanningInterfacesType_c* in, pyramid::domain_model::agra::PlanningInterfacesType& out);

void to_c(const pyramid::domain_model::agra::PlanningInterfaceType& in, pyramid_data_model_agra_PlanningInterfaceType_c* out);
void from_c(const pyramid_data_model_agra_PlanningInterfaceType_c* in, pyramid::domain_model::agra::PlanningInterfaceType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningInterfaceDetailsType& in, pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningInterfaceDetailsType_c* in, pyramid::domain_model::agra::MA_PlanningInterfaceDetailsType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanProcessType& in, pyramid_data_model_agra_MA_MissionPlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanProcessType_c* in, pyramid::domain_model::agra::MA_MissionPlanProcessType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanProcessDescriptionType& in, pyramid_data_model_agra_MA_MissionPlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanProcessDescriptionType_c* in, pyramid::domain_model::agra::MA_MissionPlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::PlanningDiscoveryBaseType& in, pyramid_data_model_agra_PlanningDiscoveryBaseType_c* out);
void from_c(const pyramid_data_model_agra_PlanningDiscoveryBaseType_c* in, pyramid::domain_model::agra::PlanningDiscoveryBaseType& out);

void to_c(const pyramid::domain_model::agra::PlanningApplicabilitySystemType& in, pyramid_data_model_agra_PlanningApplicabilitySystemType_c* out);
void from_c(const pyramid_data_model_agra_PlanningApplicabilitySystemType_c* in, pyramid::domain_model::agra::PlanningApplicabilitySystemType& out);

void to_c(const pyramid::domain_model::agra::ApplicabilityType& in, pyramid_data_model_agra_ApplicabilityType_c* out);
void from_c(const pyramid_data_model_agra_ApplicabilityType_c* in, pyramid::domain_model::agra::ApplicabilityType& out);

void to_c(const pyramid::domain_model::agra::TaskPlanProcessType& in, pyramid_data_model_agra_TaskPlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_TaskPlanProcessType_c* in, pyramid::domain_model::agra::TaskPlanProcessType& out);

void to_c(const pyramid::domain_model::agra::TaskPlanProcessDescriptionType& in, pyramid_data_model_agra_TaskPlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_TaskPlanProcessDescriptionType_c* in, pyramid::domain_model::agra::TaskPlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanProcessType& in, pyramid_data_model_agra_RoutePlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanProcessType_c* in, pyramid::domain_model::agra::RoutePlanProcessType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanProcessDescriptionType& in, pyramid_data_model_agra_RoutePlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanProcessDescriptionType_c* in, pyramid::domain_model::agra::RoutePlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::ActivityPlanProcessType& in, pyramid_data_model_agra_ActivityPlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_ActivityPlanProcessType_c* in, pyramid::domain_model::agra::ActivityPlanProcessType& out);

void to_c(const pyramid::domain_model::agra::ActivityPlanProcessDescriptionType& in, pyramid_data_model_agra_ActivityPlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_ActivityPlanProcessDescriptionType_c* in, pyramid::domain_model::agra::ActivityPlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanProcessType& in, pyramid_data_model_agra_OrbitPlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanProcessType_c* in, pyramid::domain_model::agra::OrbitPlanProcessType& out);

void to_c(const pyramid::domain_model::agra::OrbitPlanProcessDescriptionType& in, pyramid_data_model_agra_OrbitPlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_OrbitPlanProcessDescriptionType_c* in, pyramid::domain_model::agra::OrbitPlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::EffectPlanProcessType& in, pyramid_data_model_agra_EffectPlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_EffectPlanProcessType_c* in, pyramid::domain_model::agra::EffectPlanProcessType& out);

void to_c(const pyramid::domain_model::agra::EffectPlanProcessDescriptionType& in, pyramid_data_model_agra_EffectPlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_EffectPlanProcessDescriptionType_c* in, pyramid::domain_model::agra::EffectPlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::ActionPlanProcessType& in, pyramid_data_model_agra_ActionPlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_ActionPlanProcessType_c* in, pyramid::domain_model::agra::ActionPlanProcessType& out);

void to_c(const pyramid::domain_model::agra::ActionPlanProcessDescriptionType& in, pyramid_data_model_agra_ActionPlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_ActionPlanProcessDescriptionType_c* in, pyramid::domain_model::agra::ActionPlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::ResponsePlanProcessType& in, pyramid_data_model_agra_ResponsePlanProcessType_c* out);
void from_c(const pyramid_data_model_agra_ResponsePlanProcessType_c* in, pyramid::domain_model::agra::ResponsePlanProcessType& out);

void to_c(const pyramid::domain_model::agra::ResponsePlanProcessDescriptionType& in, pyramid_data_model_agra_ResponsePlanProcessDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_ResponsePlanProcessDescriptionType_c* in, pyramid::domain_model::agra::ResponsePlanProcessDescriptionType& out);

void to_c(const pyramid::domain_model::agra::SupportedPlanActivationAutonomyType& in, pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c* out);
void from_c(const pyramid_data_model_agra_SupportedPlanActivationAutonomyType_c* in, pyramid::domain_model::agra::SupportedPlanActivationAutonomyType& out);

void to_c(const pyramid::domain_model::agra::PlanScoringProcessType& in, pyramid_data_model_agra_PlanScoringProcessType_c* out);
void from_c(const pyramid_data_model_agra_PlanScoringProcessType_c* in, pyramid::domain_model::agra::PlanScoringProcessType& out);

void to_c(const pyramid::domain_model::agra::ScoringProcessID_Type& in, pyramid_data_model_agra_ScoringProcessID_Type_c* out);
void from_c(const pyramid_data_model_agra_ScoringProcessID_Type_c* in, pyramid::domain_model::agra::ScoringProcessID_Type& out);

void to_c(const pyramid::domain_model::agra::PlanScoringDescriptionType& in, pyramid_data_model_agra_PlanScoringDescriptionType_c* out);
void from_c(const pyramid_data_model_agra_PlanScoringDescriptionType_c* in, pyramid::domain_model::agra::PlanScoringDescriptionType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandMT& in, pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandMT& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandMDT& in, pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMDT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandMDT& out);

void to_c(const pyramid::domain_model::agra::CommandBaseType& in, pyramid_data_model_agra_CommandBaseType_c* out);
void from_c(const pyramid_data_model_agra_CommandBaseType_c* in, pyramid::domain_model::agra::CommandBaseType& out);

void to_c(const pyramid::domain_model::agra::PlanInterfaceCommandType& in, pyramid_data_model_agra_PlanInterfaceCommandType_c* out);
void from_c(const pyramid_data_model_agra_PlanInterfaceCommandType_c* in, pyramid::domain_model::agra::PlanInterfaceCommandType& out);

void to_c(const pyramid::domain_model::agra::PlanInterfaceStateType& in, pyramid_data_model_agra_PlanInterfaceStateType_c* out);
void from_c(const pyramid_data_model_agra_PlanInterfaceStateType_c* in, pyramid::domain_model::agra::PlanInterfaceStateType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanningAutonomySettingType& in, pyramid_data_model_agra_MA_MissionPlanningAutonomySettingType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanningAutonomySettingType_c* in, pyramid::domain_model::agra::MA_MissionPlanningAutonomySettingType& out);

void to_c(const pyramid::domain_model::agra::MA_PlannedSequentialTriggerType& in, pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlannedSequentialTriggerType_c* in, pyramid::domain_model::agra::MA_PlannedSequentialTriggerType& out);

void to_c(const pyramid::domain_model::agra::RequirementFilterType& in, pyramid_data_model_agra_RequirementFilterType_c* out);
void from_c(const pyramid_data_model_agra_RequirementFilterType_c* in, pyramid::domain_model::agra::RequirementFilterType& out);

void to_c(const pyramid::domain_model::agra::OperatorLocationOfInterestClauseType& in, pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c* out);
void from_c(const pyramid_data_model_agra_OperatorLocationOfInterestClauseType_c* in, pyramid::domain_model::agra::OperatorLocationOfInterestClauseType& out);

void to_c(const pyramid::domain_model::agra::OperatorLocationOfInterestComparativeType& in, pyramid_data_model_agra_OperatorLocationOfInterestComparativeType_c* out);
void from_c(const pyramid_data_model_agra_OperatorLocationOfInterestComparativeType_c* in, pyramid::domain_model::agra::OperatorLocationOfInterestComparativeType& out);

void to_c(const pyramid::domain_model::agra::QueryMessageType& in, pyramid_data_model_agra_QueryMessageType_c* out);
void from_c(const pyramid_data_model_agra_QueryMessageType_c* in, pyramid::domain_model::agra::QueryMessageType& out);

void to_c(const pyramid::domain_model::agra::MA_InertialStateType& in, pyramid_data_model_agra_MA_InertialStateType_c* out);
void from_c(const pyramid_data_model_agra_MA_InertialStateType_c* in, pyramid::domain_model::agra::MA_InertialStateType& out);

void to_c(const pyramid::domain_model::agra::WeatherReportGridDataType& in, pyramid_data_model_agra_WeatherReportGridDataType_c* out);
void from_c(const pyramid_data_model_agra_WeatherReportGridDataType_c* in, pyramid::domain_model::agra::WeatherReportGridDataType& out);

void to_c(const pyramid::domain_model::agra::WayPointType& in, pyramid_data_model_agra_WayPointType_c* out);
void from_c(const pyramid_data_model_agra_WayPointType_c* in, pyramid::domain_model::agra::WayPointType& out);

void to_c(const pyramid::domain_model::agra::WayPointPointChoiceType& in, pyramid_data_model_agra_WayPointPointChoiceType_c* out);
void from_c(const pyramid_data_model_agra_WayPointPointChoiceType_c* in, pyramid::domain_model::agra::WayPointPointChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanningAutonomyResponseChoiceType& in, pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_c* in, pyramid::domain_model::agra::MA_MissionPlanningAutonomyResponseChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningAllowedEscalationType& in, pyramid_data_model_agra_MA_PlanningAllowedEscalationType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningAllowedEscalationType_c* in, pyramid::domain_model::agra::MA_PlanningAllowedEscalationType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanningAutonomySettingByResultType& in, pyramid_data_model_agra_MA_MissionPlanningAutonomySettingByResultType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanningAutonomySettingByResultType_c* in, pyramid::domain_model::agra::MA_MissionPlanningAutonomySettingByResultType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanningByResultAutonomousActionType& in, pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_c* in, pyramid::domain_model::agra::MA_MissionPlanningByResultAutonomousActionType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningAllowedType& in, pyramid_data_model_agra_MA_PlanningAllowedType_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningAllowedType_c* in, pyramid::domain_model::agra::MA_PlanningAllowedType& out);

void to_c(const pyramid::domain_model::agra::MA_AutonomousPlanCommandType& in, pyramid_data_model_agra_MA_AutonomousPlanCommandType_c* out);
void from_c(const pyramid_data_model_agra_MA_AutonomousPlanCommandType_c* in, pyramid::domain_model::agra::MA_AutonomousPlanCommandType& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List& in, pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List_c* in, pyramid::domain_model::agra::MA_MissionPlanningByResultAutonomousActionType_PlanningAllowed_List& out);

void to_c(const pyramid::domain_model::agra::MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List& in, pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c* out);
void from_c(const pyramid_data_model_agra_MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List_c* in, pyramid::domain_model::agra::MA_MissionPlanningAutonomyResponseChoiceType_AutonomousPlanningAction_List& out);

void to_c(const pyramid::domain_model::agra::PlanActivationAutonomyType& in, pyramid_data_model_agra_PlanActivationAutonomyType_c* out);
void from_c(const pyramid_data_model_agra_PlanActivationAutonomyType_c* in, pyramid::domain_model::agra::PlanActivationAutonomyType& out);

void to_c(const pyramid::domain_model::agra::PlanActivationAutonomyType_BySubPlan_List& in, pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c* out);
void from_c(const pyramid_data_model_agra_PlanActivationAutonomyType_BySubPlan_List_c* in, pyramid::domain_model::agra::PlanActivationAutonomyType_BySubPlan_List& out);

void to_c(const pyramid::domain_model::agra::ContingencyPathAutonomyType& in, pyramid_data_model_agra_ContingencyPathAutonomyType_c* out);
void from_c(const pyramid_data_model_agra_ContingencyPathAutonomyType_c* in, pyramid::domain_model::agra::ContingencyPathAutonomyType& out);

void to_c(const pyramid::domain_model::agra::ContingencyPathSpacingType& in, pyramid_data_model_agra_ContingencyPathSpacingType_c* out);
void from_c(const pyramid_data_model_agra_ContingencyPathSpacingType_c* in, pyramid::domain_model::agra::ContingencyPathSpacingType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMT& in, pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMDT& in, pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMDT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMDT& out);

void to_c(const pyramid::domain_model::agra::CommandStatusBaseType& in, pyramid_data_model_agra_CommandStatusBaseType_c* out);
void from_c(const pyramid_data_model_agra_CommandStatusBaseType_c* in, pyramid::domain_model::agra::CommandStatusBaseType& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionStatusMT& in, pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_PlanningFunctionStatusMDT& in, pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_PlanningFunctionStatusMDT_c* in, pyramid::domain_model::agra::MA_PlanningFunctionStatusMDT& out);

void to_c(const pyramid::domain_model::agra::MA_ResponseMT& in, pyramid_data_model_agra_MA_ResponseMT_c* out);
void from_c(const pyramid_data_model_agra_MA_ResponseMT_c* in, pyramid::domain_model::agra::MA_ResponseMT& out);

void to_c(const pyramid::domain_model::agra::MA_ResponseMDT& in, pyramid_data_model_agra_MA_ResponseMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_ResponseMDT_c* in, pyramid::domain_model::agra::MA_ResponseMDT& out);

void to_c(const pyramid::domain_model::agra::MA_ResponseOptionDetailsType& in, pyramid_data_model_agra_MA_ResponseOptionDetailsType_c* out);
void from_c(const pyramid_data_model_agra_MA_ResponseOptionDetailsType_c* in, pyramid::domain_model::agra::MA_ResponseOptionDetailsType& out);

void to_c(const pyramid::domain_model::agra::MA_RuleResponseType& in, pyramid_data_model_agra_MA_RuleResponseType_c* out);
void from_c(const pyramid_data_model_agra_MA_RuleResponseType_c* in, pyramid::domain_model::agra::MA_RuleResponseType& out);

void to_c(const pyramid::domain_model::agra::ResponseTemplateType& in, pyramid_data_model_agra_ResponseTemplateType_c* out);
void from_c(const pyramid_data_model_agra_ResponseTemplateType_c* in, pyramid::domain_model::agra::ResponseTemplateType& out);

void to_c(const pyramid::domain_model::agra::RequirementsTemplateID_Type& in, pyramid_data_model_agra_RequirementsTemplateID_Type_c* out);
void from_c(const pyramid_data_model_agra_RequirementsTemplateID_Type_c* in, pyramid::domain_model::agra::RequirementsTemplateID_Type& out);

void to_c(const pyramid::domain_model::agra::PlanInputsCoreType& in, pyramid_data_model_agra_PlanInputsCoreType_c* out);
void from_c(const pyramid_data_model_agra_PlanInputsCoreType_c* in, pyramid::domain_model::agra::PlanInputsCoreType& out);

void to_c(const pyramid::domain_model::agra::ReplanReasonType& in, pyramid_data_model_agra_ReplanReasonType_c* out);
void from_c(const pyramid_data_model_agra_ReplanReasonType_c* in, pyramid::domain_model::agra::ReplanReasonType& out);

void to_c(const pyramid::domain_model::agra::PlanningTriggerType& in, pyramid_data_model_agra_PlanningTriggerType_c* out);
void from_c(const pyramid_data_model_agra_PlanningTriggerType_c* in, pyramid::domain_model::agra::PlanningTriggerType& out);

void to_c(const pyramid::domain_model::agra::MissionEnvironmentConstraintType& in, pyramid_data_model_agra_MissionEnvironmentConstraintType_c* out);
void from_c(const pyramid_data_model_agra_MissionEnvironmentConstraintType_c* in, pyramid::domain_model::agra::MissionEnvironmentConstraintType& out);

void to_c(const pyramid::domain_model::agra::RequirementRiskAdjustmentType& in, pyramid_data_model_agra_RequirementRiskAdjustmentType_c* out);
void from_c(const pyramid_data_model_agra_RequirementRiskAdjustmentType_c* in, pyramid::domain_model::agra::RequirementRiskAdjustmentType& out);

void to_c(const pyramid::domain_model::agra::RequirementChoiceType& in, pyramid_data_model_agra_RequirementChoiceType_c* out);
void from_c(const pyramid_data_model_agra_RequirementChoiceType_c* in, pyramid::domain_model::agra::RequirementChoiceType& out);

void to_c(const pyramid::domain_model::agra::RequirementTaxonomyChoiceType& in, pyramid_data_model_agra_RequirementTaxonomyChoiceType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTaxonomyChoiceType_c* in, pyramid::domain_model::agra::RequirementTaxonomyChoiceType& out);

void to_c(const pyramid::domain_model::agra::ResponseAlertType& in, pyramid_data_model_agra_ResponseAlertType_c* out);
void from_c(const pyramid_data_model_agra_ResponseAlertType_c* in, pyramid::domain_model::agra::ResponseAlertType& out);

void to_c(const pyramid::domain_model::agra::OperatorRecommendationType& in, pyramid_data_model_agra_OperatorRecommendationType_c* out);
void from_c(const pyramid_data_model_agra_OperatorRecommendationType_c* in, pyramid::domain_model::agra::OperatorRecommendationType& out);

void to_c(const pyramid::domain_model::agra::RequirementsTemplateType& in, pyramid_data_model_agra_RequirementsTemplateType_c* out);
void from_c(const pyramid_data_model_agra_RequirementsTemplateType_c* in, pyramid::domain_model::agra::RequirementsTemplateType& out);

void to_c(const pyramid::domain_model::agra::RequirementTemplateOptionsType& in, pyramid_data_model_agra_RequirementTemplateOptionsType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTemplateOptionsType_c* in, pyramid::domain_model::agra::RequirementTemplateOptionsType& out);

void to_c(const pyramid::domain_model::agra::RequirementTemplateOptionType& in, pyramid_data_model_agra_RequirementTemplateOptionType_c* out);
void from_c(const pyramid_data_model_agra_RequirementTemplateOptionType_c* in, pyramid::domain_model::agra::RequirementTemplateOptionType& out);

void to_c(const pyramid::domain_model::agra::ResponseOptionType& in, pyramid_data_model_agra_ResponseOptionType_c* out);
void from_c(const pyramid_data_model_agra_ResponseOptionType_c* in, pyramid::domain_model::agra::ResponseOptionType& out);

void to_c(const pyramid::domain_model::agra::TaskResponseType& in, pyramid_data_model_agra_TaskResponseType_c* out);
void from_c(const pyramid_data_model_agra_TaskResponseType_c* in, pyramid::domain_model::agra::TaskResponseType& out);

void to_c(const pyramid::domain_model::agra::AirSampleTaskBaseType& in, pyramid_data_model_agra_AirSampleTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_AirSampleTaskBaseType_c* in, pyramid::domain_model::agra::AirSampleTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::ProductOutputCommandBasicType& in, pyramid_data_model_agra_ProductOutputCommandBasicType_c* out);
void from_c(const pyramid_data_model_agra_ProductOutputCommandBasicType_c* in, pyramid::domain_model::agra::ProductOutputCommandBasicType& out);

void to_c(const pyramid::domain_model::agra::ProductOutputType& in, pyramid_data_model_agra_ProductOutputType_c* out);
void from_c(const pyramid_data_model_agra_ProductOutputType_c* in, pyramid::domain_model::agra::ProductOutputType& out);

void to_c(const pyramid::domain_model::agra::FileFormatType& in, pyramid_data_model_agra_FileFormatType_c* out);
void from_c(const pyramid_data_model_agra_FileFormatType_c* in, pyramid::domain_model::agra::FileFormatType& out);

void to_c(const pyramid::domain_model::agra::AMTI_TaskBaseType& in, pyramid_data_model_agra_AMTI_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_AMTI_TaskBaseType_c* in, pyramid::domain_model::agra::AMTI_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::AMTI_CollectionConstraintsType& in, pyramid_data_model_agra_AMTI_CollectionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_AMTI_CollectionConstraintsType_c* in, pyramid::domain_model::agra::AMTI_CollectionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::CollectionConstraintsType& in, pyramid_data_model_agra_CollectionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_CollectionConstraintsType_c* in, pyramid::domain_model::agra::CollectionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::AngleQuarterPairType& in, pyramid_data_model_agra_AngleQuarterPairType_c* out);
void from_c(const pyramid_data_model_agra_AngleQuarterPairType_c* in, pyramid::domain_model::agra::AngleQuarterPairType& out);

void to_c(const pyramid::domain_model::agra::EmconConstraintType& in, pyramid_data_model_agra_EmconConstraintType_c* out);
void from_c(const pyramid_data_model_agra_EmconConstraintType_c* in, pyramid::domain_model::agra::EmconConstraintType& out);

void to_c(const pyramid::domain_model::agra::EmconOverrideType& in, pyramid_data_model_agra_EmconOverrideType_c* out);
void from_c(const pyramid_data_model_agra_EmconOverrideType_c* in, pyramid::domain_model::agra::EmconOverrideType& out);

void to_c(const pyramid::domain_model::agra::EmconERP_Type& in, pyramid_data_model_agra_EmconERP_Type_c* out);
void from_c(const pyramid_data_model_agra_EmconERP_Type_c* in, pyramid::domain_model::agra::EmconERP_Type& out);

void to_c(const pyramid::domain_model::agra::SpeedRangeType& in, pyramid_data_model_agra_SpeedRangeType_c* out);
void from_c(const pyramid_data_model_agra_SpeedRangeType_c* in, pyramid::domain_model::agra::SpeedRangeType& out);

void to_c(const pyramid::domain_model::agra::AO_TaskBaseType& in, pyramid_data_model_agra_AO_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_AO_TaskBaseType_c* in, pyramid::domain_model::agra::AO_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::OpticalCollectionConstraintsType& in, pyramid_data_model_agra_OpticalCollectionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_OpticalCollectionConstraintsType_c* in, pyramid::domain_model::agra::OpticalCollectionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::GimbalOrientationPairType& in, pyramid_data_model_agra_GimbalOrientationPairType_c* out);
void from_c(const pyramid_data_model_agra_GimbalOrientationPairType_c* in, pyramid::domain_model::agra::GimbalOrientationPairType& out);

void to_c(const pyramid::domain_model::agra::ComponentID_Type& in, pyramid_data_model_agra_ComponentID_Type_c* out);
void from_c(const pyramid_data_model_agra_ComponentID_Type_c* in, pyramid::domain_model::agra::ComponentID_Type& out);

void to_c(const pyramid::domain_model::agra::GimbalAxisPairType& in, pyramid_data_model_agra_GimbalAxisPairType_c* out);
void from_c(const pyramid_data_model_agra_GimbalAxisPairType_c* in, pyramid::domain_model::agra::GimbalAxisPairType& out);

void to_c(const pyramid::domain_model::agra::GimbalAxisID_Type& in, pyramid_data_model_agra_GimbalAxisID_Type_c* out);
void from_c(const pyramid_data_model_agra_GimbalAxisID_Type_c* in, pyramid::domain_model::agra::GimbalAxisID_Type& out);

void to_c(const pyramid::domain_model::agra::COMINT_TaskBaseType& in, pyramid_data_model_agra_COMINT_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_TaskBaseType_c* in, pyramid::domain_model::agra::COMINT_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::CommRelayTaskBaseType& in, pyramid_data_model_agra_CommRelayTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_CommRelayTaskBaseType_c* in, pyramid::domain_model::agra::CommRelayTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::FrequencySetType& in, pyramid_data_model_agra_FrequencySetType_c* out);
void from_c(const pyramid_data_model_agra_FrequencySetType_c* in, pyramid::domain_model::agra::FrequencySetType& out);

void to_c(const pyramid::domain_model::agra::FrequencyMultiChannelType& in, pyramid_data_model_agra_FrequencyMultiChannelType_c* out);
void from_c(const pyramid_data_model_agra_FrequencyMultiChannelType_c* in, pyramid::domain_model::agra::FrequencyMultiChannelType& out);

void to_c(const pyramid::domain_model::agra::EA_ResponseType& in, pyramid_data_model_agra_EA_ResponseType_c* out);
void from_c(const pyramid_data_model_agra_EA_ResponseType_c* in, pyramid::domain_model::agra::EA_ResponseType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskEscortType& in, pyramid_data_model_agra_EA_TaskEscortType_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskEscortType_c* in, pyramid::domain_model::agra::EA_TaskEscortType& out);

void to_c(const pyramid::domain_model::agra::ProtectedAssetType& in, pyramid_data_model_agra_ProtectedAssetType_c* out);
void from_c(const pyramid_data_model_agra_ProtectedAssetType_c* in, pyramid::domain_model::agra::ProtectedAssetType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskProtectedAssetsType& in, pyramid_data_model_agra_EA_TaskProtectedAssetsType_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskProtectedAssetsType_c* in, pyramid::domain_model::agra::EA_TaskProtectedAssetsType& out);

void to_c(const pyramid::domain_model::agra::ProtectedAssetAndThreatType& in, pyramid_data_model_agra_ProtectedAssetAndThreatType_c* out);
void from_c(const pyramid_data_model_agra_ProtectedAssetAndThreatType_c* in, pyramid::domain_model::agra::ProtectedAssetAndThreatType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskThreatsType& in, pyramid_data_model_agra_EA_TaskThreatsType_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskThreatsType_c* in, pyramid::domain_model::agra::EA_TaskThreatsType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskSuppressionConstraintsType& in, pyramid_data_model_agra_EA_TaskSuppressionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskSuppressionConstraintsType_c* in, pyramid::domain_model::agra::EA_TaskSuppressionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::EA_TargetType& in, pyramid_data_model_agra_EA_TargetType_c* out);
void from_c(const pyramid_data_model_agra_EA_TargetType_c* in, pyramid::domain_model::agra::EA_TargetType& out);

void to_c(const pyramid::domain_model::agra::EA_EmitterDataType& in, pyramid_data_model_agra_EA_EmitterDataType_c* out);
void from_c(const pyramid_data_model_agra_EA_EmitterDataType_c* in, pyramid::domain_model::agra::EA_EmitterDataType& out);

void to_c(const pyramid::domain_model::agra::EA_TargetPointingType& in, pyramid_data_model_agra_EA_TargetPointingType_c* out);
void from_c(const pyramid_data_model_agra_EA_TargetPointingType_c* in, pyramid::domain_model::agra::EA_TargetPointingType& out);

void to_c(const pyramid::domain_model::agra::AirVolumeSensorReferencedType& in, pyramid_data_model_agra_AirVolumeSensorReferencedType_c* out);
void from_c(const pyramid_data_model_agra_AirVolumeSensorReferencedType_c* in, pyramid::domain_model::agra::AirVolumeSensorReferencedType& out);

void to_c(const pyramid::domain_model::agra::AltitudeRangePairType& in, pyramid_data_model_agra_AltitudeRangePairType_c* out);
void from_c(const pyramid_data_model_agra_AltitudeRangePairType_c* in, pyramid::domain_model::agra::AltitudeRangePairType& out);

void to_c(const pyramid::domain_model::agra::EA_PowerType& in, pyramid_data_model_agra_EA_PowerType_c* out);
void from_c(const pyramid_data_model_agra_EA_PowerType_c* in, pyramid::domain_model::agra::EA_PowerType& out);

void to_c(const pyramid::domain_model::agra::EA_TechniqueIdentifierType& in, pyramid_data_model_agra_EA_TechniqueIdentifierType_c* out);
void from_c(const pyramid_data_model_agra_EA_TechniqueIdentifierType_c* in, pyramid::domain_model::agra::EA_TechniqueIdentifierType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskThreatsType_SuppressionConstraints_List& in, pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskThreatsType_SuppressionConstraints_List_c* in, pyramid::domain_model::agra::EA_TaskThreatsType_SuppressionConstraints_List& out);

void to_c(const pyramid::domain_model::agra::ESM_TaskBaseType& in, pyramid_data_model_agra_ESM_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_ESM_TaskBaseType_c* in, pyramid::domain_model::agra::ESM_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::FlightTaskBaseType& in, pyramid_data_model_agra_FlightTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_FlightTaskBaseType_c* in, pyramid::domain_model::agra::FlightTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::OrbitChangeTaskBaseType& in, pyramid_data_model_agra_OrbitChangeTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_OrbitChangeTaskBaseType_c* in, pyramid::domain_model::agra::OrbitChangeTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceTaskBaseType& in, pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceTaskBaseType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSubCapabilityDetailsChoiceType& in, pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSubCapabilityDetailsChoiceType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSubCapabilityDetailsChoiceType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSearchType& in, pyramid_data_model_agra_OrbitalSurveillanceSearchType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSearchType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSearchType& out);

void to_c(const pyramid::domain_model::agra::OrbitAccuracyType& in, pyramid_data_model_agra_OrbitAccuracyType_c* out);
void from_c(const pyramid_data_model_agra_OrbitAccuracyType_c* in, pyramid::domain_model::agra::OrbitAccuracyType& out);

void to_c(const pyramid::domain_model::agra::CharacterizationObjectiveType& in, pyramid_data_model_agra_CharacterizationObjectiveType_c* out);
void from_c(const pyramid_data_model_agra_CharacterizationObjectiveType_c* in, pyramid::domain_model::agra::CharacterizationObjectiveType& out);

void to_c(const pyramid::domain_model::agra::CharacterizationOptionsType& in, pyramid_data_model_agra_CharacterizationOptionsType_c* out);
void from_c(const pyramid_data_model_agra_CharacterizationOptionsType_c* in, pyramid::domain_model::agra::CharacterizationOptionsType& out);

void to_c(const pyramid::domain_model::agra::CharacterizationChoiceType& in, pyramid_data_model_agra_CharacterizationChoiceType_c* out);
void from_c(const pyramid_data_model_agra_CharacterizationChoiceType_c* in, pyramid::domain_model::agra::CharacterizationChoiceType& out);

void to_c(const pyramid::domain_model::agra::FrequencyParamsType& in, pyramid_data_model_agra_FrequencyParamsType_c* out);
void from_c(const pyramid_data_model_agra_FrequencyParamsType_c* in, pyramid::domain_model::agra::FrequencyParamsType& out);

void to_c(const pyramid::domain_model::agra::IR_ImageParamsType& in, pyramid_data_model_agra_IR_ImageParamsType_c* out);
void from_c(const pyramid_data_model_agra_IR_ImageParamsType_c* in, pyramid::domain_model::agra::IR_ImageParamsType& out);

void to_c(const pyramid::domain_model::agra::MetricParamsType& in, pyramid_data_model_agra_MetricParamsType_c* out);
void from_c(const pyramid_data_model_agra_MetricParamsType_c* in, pyramid::domain_model::agra::MetricParamsType& out);

void to_c(const pyramid::domain_model::agra::Narrowband_SOI_ParamsType& in, pyramid_data_model_agra_Narrowband_SOI_ParamsType_c* out);
void from_c(const pyramid_data_model_agra_Narrowband_SOI_ParamsType_c* in, pyramid::domain_model::agra::Narrowband_SOI_ParamsType& out);

void to_c(const pyramid::domain_model::agra::OpticalImageParamsType& in, pyramid_data_model_agra_OpticalImageParamsType_c* out);
void from_c(const pyramid_data_model_agra_OpticalImageParamsType_c* in, pyramid::domain_model::agra::OpticalImageParamsType& out);

void to_c(const pyramid::domain_model::agra::ProductResolutionType& in, pyramid_data_model_agra_ProductResolutionType_c* out);
void from_c(const pyramid_data_model_agra_ProductResolutionType_c* in, pyramid::domain_model::agra::ProductResolutionType& out);

void to_c(const pyramid::domain_model::agra::RCS_ParamsType& in, pyramid_data_model_agra_RCS_ParamsType_c* out);
void from_c(const pyramid_data_model_agra_RCS_ParamsType_c* in, pyramid::domain_model::agra::RCS_ParamsType& out);

void to_c(const pyramid::domain_model::agra::VisMagParamsType& in, pyramid_data_model_agra_VisMagParamsType_c* out);
void from_c(const pyramid_data_model_agra_VisMagParamsType_c* in, pyramid::domain_model::agra::VisMagParamsType& out);

void to_c(const pyramid::domain_model::agra::Wideband_SOI_ParamsType& in, pyramid_data_model_agra_Wideband_SOI_ParamsType_c* out);
void from_c(const pyramid_data_model_agra_Wideband_SOI_ParamsType_c* in, pyramid::domain_model::agra::Wideband_SOI_ParamsType& out);

void to_c(const pyramid::domain_model::agra::RangeResolutionType& in, pyramid_data_model_agra_RangeResolutionType_c* out);
void from_c(const pyramid_data_model_agra_RangeResolutionType_c* in, pyramid::domain_model::agra::RangeResolutionType& out);

void to_c(const pyramid::domain_model::agra::PhotometryParamsType& in, pyramid_data_model_agra_PhotometryParamsType_c* out);
void from_c(const pyramid_data_model_agra_PhotometryParamsType_c* in, pyramid::domain_model::agra::PhotometryParamsType& out);

void to_c(const pyramid::domain_model::agra::ColorPhotometryParamsType& in, pyramid_data_model_agra_ColorPhotometryParamsType_c* out);
void from_c(const pyramid_data_model_agra_ColorPhotometryParamsType_c* in, pyramid::domain_model::agra::ColorPhotometryParamsType& out);

void to_c(const pyramid::domain_model::agra::StabilityCharacterizationType& in, pyramid_data_model_agra_StabilityCharacterizationType_c* out);
void from_c(const pyramid_data_model_agra_StabilityCharacterizationType_c* in, pyramid::domain_model::agra::StabilityCharacterizationType& out);

void to_c(const pyramid::domain_model::agra::StructureAssessmentType& in, pyramid_data_model_agra_StructureAssessmentType_c* out);
void from_c(const pyramid_data_model_agra_StructureAssessmentType_c* in, pyramid::domain_model::agra::StructureAssessmentType& out);

void to_c(const pyramid::domain_model::agra::SizeEstimationType& in, pyramid_data_model_agra_SizeEstimationType_c* out);
void from_c(const pyramid_data_model_agra_SizeEstimationType_c* in, pyramid::domain_model::agra::SizeEstimationType& out);

void to_c(const pyramid::domain_model::agra::ResolvedCharacterizationType& in, pyramid_data_model_agra_ResolvedCharacterizationType_c* out);
void from_c(const pyramid_data_model_agra_ResolvedCharacterizationType_c* in, pyramid::domain_model::agra::ResolvedCharacterizationType& out);

void to_c(const pyramid::domain_model::agra::DistanceResolutionSpecificationType& in, pyramid_data_model_agra_DistanceResolutionSpecificationType_c* out);
void from_c(const pyramid_data_model_agra_DistanceResolutionSpecificationType_c* in, pyramid::domain_model::agra::DistanceResolutionSpecificationType& out);

void to_c(const pyramid::domain_model::agra::ResolvedCharacterizationAspectCoverageType& in, pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c* out);
void from_c(const pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_c* in, pyramid::domain_model::agra::ResolvedCharacterizationAspectCoverageType& out);

void to_c(const pyramid::domain_model::agra::ResolvedCharacterizationAspectCoverageType_BodyReference_List& in, pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c* out);
void from_c(const pyramid_data_model_agra_ResolvedCharacterizationAspectCoverageType_BodyReference_List_c* in, pyramid::domain_model::agra::ResolvedCharacterizationAspectCoverageType_BodyReference_List& out);

void to_c(const pyramid::domain_model::agra::IdentificationVerificationType& in, pyramid_data_model_agra_IdentificationVerificationType_c* out);
void from_c(const pyramid_data_model_agra_IdentificationVerificationType_c* in, pyramid::domain_model::agra::IdentificationVerificationType& out);

void to_c(const pyramid::domain_model::agra::SatelliteOperationsChangesCharacterizationType& in, pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c* out);
void from_c(const pyramid_data_model_agra_SatelliteOperationsChangesCharacterizationType_c* in, pyramid::domain_model::agra::SatelliteOperationsChangesCharacterizationType& out);

void to_c(const pyramid::domain_model::agra::MultiObjectType& in, pyramid_data_model_agra_MultiObjectType_c* out);
void from_c(const pyramid_data_model_agra_MultiObjectType_c* in, pyramid::domain_model::agra::MultiObjectType& out);

void to_c(const pyramid::domain_model::agra::ManeuverDetectionType& in, pyramid_data_model_agra_ManeuverDetectionType_c* out);
void from_c(const pyramid_data_model_agra_ManeuverDetectionType_c* in, pyramid::domain_model::agra::ManeuverDetectionType& out);

void to_c(const pyramid::domain_model::agra::ManeuverConstraintsChoiceType& in, pyramid_data_model_agra_ManeuverConstraintsChoiceType_c* out);
void from_c(const pyramid_data_model_agra_ManeuverConstraintsChoiceType_c* in, pyramid::domain_model::agra::ManeuverConstraintsChoiceType& out);

void to_c(const pyramid::domain_model::agra::BasicManeuverConstraintsType& in, pyramid_data_model_agra_BasicManeuverConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_BasicManeuverConstraintsType_c* in, pyramid::domain_model::agra::BasicManeuverConstraintsType& out);

void to_c(const pyramid::domain_model::agra::OrbitalManeuverDetailsType& in, pyramid_data_model_agra_OrbitalManeuverDetailsType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalManeuverDetailsType_c* in, pyramid::domain_model::agra::OrbitalManeuverDetailsType& out);

void to_c(const pyramid::domain_model::agra::DeploymentDetectionType& in, pyramid_data_model_agra_DeploymentDetectionType_c* out);
void from_c(const pyramid_data_model_agra_DeploymentDetectionType_c* in, pyramid::domain_model::agra::DeploymentDetectionType& out);

void to_c(const pyramid::domain_model::agra::ProductNeededByType& in, pyramid_data_model_agra_ProductNeededByType_c* out);
void from_c(const pyramid_data_model_agra_ProductNeededByType_c* in, pyramid::domain_model::agra::ProductNeededByType& out);

void to_c(const pyramid::domain_model::agra::AllowableSensorsType& in, pyramid_data_model_agra_AllowableSensorsType_c* out);
void from_c(const pyramid_data_model_agra_AllowableSensorsType_c* in, pyramid::domain_model::agra::AllowableSensorsType& out);

void to_c(const pyramid::domain_model::agra::SensorCountConstraintType& in, pyramid_data_model_agra_SensorCountConstraintType_c* out);
void from_c(const pyramid_data_model_agra_SensorCountConstraintType_c* in, pyramid::domain_model::agra::SensorCountConstraintType& out);

void to_c(const pyramid::domain_model::agra::SensorConstraintsType& in, pyramid_data_model_agra_SensorConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_SensorConstraintsType_c* in, pyramid::domain_model::agra::SensorConstraintsType& out);

void to_c(const pyramid::domain_model::agra::SensorConstraintsBaseType& in, pyramid_data_model_agra_SensorConstraintsBaseType_c* out);
void from_c(const pyramid_data_model_agra_SensorConstraintsBaseType_c* in, pyramid::domain_model::agra::SensorConstraintsBaseType& out);

void to_c(const pyramid::domain_model::agra::SDA_SpecialInstructionsConstraintType& in, pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c* out);
void from_c(const pyramid_data_model_agra_SDA_SpecialInstructionsConstraintType_c* in, pyramid::domain_model::agra::SDA_SpecialInstructionsConstraintType& out);

void to_c(const pyramid::domain_model::agra::SDA_SpecialInstructionsSetType& in, pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c* out);
void from_c(const pyramid_data_model_agra_SDA_SpecialInstructionsSetType_c* in, pyramid::domain_model::agra::SDA_SpecialInstructionsSetType& out);

void to_c(const pyramid::domain_model::agra::PO_TaskBaseType& in, pyramid_data_model_agra_PO_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_PO_TaskBaseType_c* in, pyramid::domain_model::agra::PO_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::PO_ProductOutputCommandImageryType& in, pyramid_data_model_agra_PO_ProductOutputCommandImageryType_c* out);
void from_c(const pyramid_data_model_agra_PO_ProductOutputCommandImageryType_c* in, pyramid::domain_model::agra::PO_ProductOutputCommandImageryType& out);

void to_c(const pyramid::domain_model::agra::PO_ProductGeneratorOutputID_Type& in, pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c* out);
void from_c(const pyramid_data_model_agra_PO_ProductGeneratorOutputID_Type_c* in, pyramid::domain_model::agra::PO_ProductGeneratorOutputID_Type& out);

void to_c(const pyramid::domain_model::agra::JPEG_SettingsType& in, pyramid_data_model_agra_JPEG_SettingsType_c* out);
void from_c(const pyramid_data_model_agra_JPEG_SettingsType_c* in, pyramid::domain_model::agra::JPEG_SettingsType& out);

void to_c(const pyramid::domain_model::agra::JPEG_WaveletTransformType& in, pyramid_data_model_agra_JPEG_WaveletTransformType_c* out);
void from_c(const pyramid_data_model_agra_JPEG_WaveletTransformType_c* in, pyramid::domain_model::agra::JPEG_WaveletTransformType& out);

void to_c(const pyramid::domain_model::agra::NITF_PackingPlanPET& in, pyramid_data_model_agra_NITF_PackingPlanPET_c* out);
void from_c(const pyramid_data_model_agra_NITF_PackingPlanPET_c* in, pyramid::domain_model::agra::NITF_PackingPlanPET& out);

void to_c(const pyramid::domain_model::agra::MISP_PackingPlanType& in, pyramid_data_model_agra_MISP_PackingPlanType_c* out);
void from_c(const pyramid_data_model_agra_MISP_PackingPlanType_c* in, pyramid::domain_model::agra::MISP_PackingPlanType& out);

void to_c(const pyramid::domain_model::agra::UMID_VideoID_Type& in, pyramid_data_model_agra_UMID_VideoID_Type_c* out);
void from_c(const pyramid_data_model_agra_UMID_VideoID_Type_c* in, pyramid::domain_model::agra::UMID_VideoID_Type& out);

void to_c(const pyramid::domain_model::agra::UMID_AudioID_Type& in, pyramid_data_model_agra_UMID_AudioID_Type_c* out);
void from_c(const pyramid_data_model_agra_UMID_AudioID_Type_c* in, pyramid::domain_model::agra::UMID_AudioID_Type& out);

void to_c(const pyramid::domain_model::agra::UMID_DataID_Type& in, pyramid_data_model_agra_UMID_DataID_Type_c* out);
void from_c(const pyramid_data_model_agra_UMID_DataID_Type_c* in, pyramid::domain_model::agra::UMID_DataID_Type& out);

void to_c(const pyramid::domain_model::agra::UMID_SystemID_Type& in, pyramid_data_model_agra_UMID_SystemID_Type_c* out);
void from_c(const pyramid_data_model_agra_UMID_SystemID_Type_c* in, pyramid::domain_model::agra::UMID_SystemID_Type& out);

void to_c(const pyramid::domain_model::agra::VideoOutputSettingsType& in, pyramid_data_model_agra_VideoOutputSettingsType_c* out);
void from_c(const pyramid_data_model_agra_VideoOutputSettingsType_c* in, pyramid::domain_model::agra::VideoOutputSettingsType& out);

void to_c(const pyramid::domain_model::agra::VideoEncoderOutputType& in, pyramid_data_model_agra_VideoEncoderOutputType_c* out);
void from_c(const pyramid_data_model_agra_VideoEncoderOutputType_c* in, pyramid::domain_model::agra::VideoEncoderOutputType& out);

void to_c(const pyramid::domain_model::agra::IP_ConnectionChoiceType& in, pyramid_data_model_agra_IP_ConnectionChoiceType_c* out);
void from_c(const pyramid_data_model_agra_IP_ConnectionChoiceType_c* in, pyramid::domain_model::agra::IP_ConnectionChoiceType& out);

void to_c(const pyramid::domain_model::agra::IPv4_ConnectionType& in, pyramid_data_model_agra_IPv4_ConnectionType_c* out);
void from_c(const pyramid_data_model_agra_IPv4_ConnectionType_c* in, pyramid::domain_model::agra::IPv4_ConnectionType& out);

void to_c(const pyramid::domain_model::agra::IPv6_ConnectionType& in, pyramid_data_model_agra_IPv6_ConnectionType_c* out);
void from_c(const pyramid_data_model_agra_IPv6_ConnectionType_c* in, pyramid::domain_model::agra::IPv6_ConnectionType& out);

void to_c(const pyramid::domain_model::agra::FileNameAndOutputType& in, pyramid_data_model_agra_FileNameAndOutputType_c* out);
void from_c(const pyramid_data_model_agra_FileNameAndOutputType_c* in, pyramid::domain_model::agra::FileNameAndOutputType& out);

void to_c(const pyramid::domain_model::agra::FileOutputType& in, pyramid_data_model_agra_FileOutputType_c* out);
void from_c(const pyramid_data_model_agra_FileOutputType_c* in, pyramid::domain_model::agra::FileOutputType& out);

void to_c(const pyramid::domain_model::agra::VideoEncoderSettingsType& in, pyramid_data_model_agra_VideoEncoderSettingsType_c* out);
void from_c(const pyramid_data_model_agra_VideoEncoderSettingsType_c* in, pyramid::domain_model::agra::VideoEncoderSettingsType& out);

void to_c(const pyramid::domain_model::agra::UnsignedIntegerMinMaxType& in, pyramid_data_model_agra_UnsignedIntegerMinMaxType_c* out);
void from_c(const pyramid_data_model_agra_UnsignedIntegerMinMaxType_c* in, pyramid::domain_model::agra::UnsignedIntegerMinMaxType& out);

void to_c(const pyramid::domain_model::agra::CropType& in, pyramid_data_model_agra_CropType_c* out);
void from_c(const pyramid_data_model_agra_CropType_c* in, pyramid::domain_model::agra::CropType& out);

void to_c(const pyramid::domain_model::agra::CropSettingsType& in, pyramid_data_model_agra_CropSettingsType_c* out);
void from_c(const pyramid_data_model_agra_CropSettingsType_c* in, pyramid::domain_model::agra::CropSettingsType& out);

void to_c(const pyramid::domain_model::agra::RefuelTaskBaseType& in, pyramid_data_model_agra_RefuelTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_RefuelTaskBaseType_c* in, pyramid::domain_model::agra::RefuelTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::SAR_TaskBaseType& in, pyramid_data_model_agra_SAR_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_SAR_TaskBaseType_c* in, pyramid::domain_model::agra::SAR_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::SAR_CollectionOptionsType& in, pyramid_data_model_agra_SAR_CollectionOptionsType_c* out);
void from_c(const pyramid_data_model_agra_SAR_CollectionOptionsType_c* in, pyramid::domain_model::agra::SAR_CollectionOptionsType& out);

void to_c(const pyramid::domain_model::agra::RadarCollectionOptionsType& in, pyramid_data_model_agra_RadarCollectionOptionsType_c* out);
void from_c(const pyramid_data_model_agra_RadarCollectionOptionsType_c* in, pyramid::domain_model::agra::RadarCollectionOptionsType& out);

void to_c(const pyramid::domain_model::agra::RadarSpoilTaperType& in, pyramid_data_model_agra_RadarSpoilTaperType_c* out);
void from_c(const pyramid_data_model_agra_RadarSpoilTaperType_c* in, pyramid::domain_model::agra::RadarSpoilTaperType& out);

void to_c(const pyramid::domain_model::agra::RadarTaperType& in, pyramid_data_model_agra_RadarTaperType_c* out);
void from_c(const pyramid_data_model_agra_RadarTaperType_c* in, pyramid::domain_model::agra::RadarTaperType& out);

void to_c(const pyramid::domain_model::agra::RadarTaperWeightingFunctionType& in, pyramid_data_model_agra_RadarTaperWeightingFunctionType_c* out);
void from_c(const pyramid_data_model_agra_RadarTaperWeightingFunctionType_c* in, pyramid::domain_model::agra::RadarTaperWeightingFunctionType& out);

void to_c(const pyramid::domain_model::agra::RadarSpoilType& in, pyramid_data_model_agra_RadarSpoilType_c* out);
void from_c(const pyramid_data_model_agra_RadarSpoilType_c* in, pyramid::domain_model::agra::RadarSpoilType& out);

void to_c(const pyramid::domain_model::agra::ElectronicProtectionOptionsEnableType& in, pyramid_data_model_agra_ElectronicProtectionOptionsEnableType_c* out);
void from_c(const pyramid_data_model_agra_ElectronicProtectionOptionsEnableType_c* in, pyramid::domain_model::agra::ElectronicProtectionOptionsEnableType& out);

void to_c(const pyramid::domain_model::agra::SupportedResolutionID_Type& in, pyramid_data_model_agra_SupportedResolutionID_Type_c* out);
void from_c(const pyramid_data_model_agra_SupportedResolutionID_Type_c* in, pyramid::domain_model::agra::SupportedResolutionID_Type& out);

void to_c(const pyramid::domain_model::agra::SAR_CollectionConstraintsType& in, pyramid_data_model_agra_SAR_CollectionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_SAR_CollectionConstraintsType_c* in, pyramid::domain_model::agra::SAR_CollectionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::SAR_CollectionConstraintsQualityType& in, pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c* out);
void from_c(const pyramid_data_model_agra_SAR_CollectionConstraintsQualityType_c* in, pyramid::domain_model::agra::SAR_CollectionConstraintsQualityType& out);

void to_c(const pyramid::domain_model::agra::PositionLocationUncertaintyType& in, pyramid_data_model_agra_PositionLocationUncertaintyType_c* out);
void from_c(const pyramid_data_model_agra_PositionLocationUncertaintyType_c* in, pyramid::domain_model::agra::PositionLocationUncertaintyType& out);

void to_c(const pyramid::domain_model::agra::SAR_WaveformType& in, pyramid_data_model_agra_SAR_WaveformType_c* out);
void from_c(const pyramid_data_model_agra_SAR_WaveformType_c* in, pyramid::domain_model::agra::SAR_WaveformType& out);

void to_c(const pyramid::domain_model::agra::ProductOutputCommandImageryType& in, pyramid_data_model_agra_ProductOutputCommandImageryType_c* out);
void from_c(const pyramid_data_model_agra_ProductOutputCommandImageryType_c* in, pyramid::domain_model::agra::ProductOutputCommandImageryType& out);

void to_c(const pyramid::domain_model::agra::SMTI_TaskBaseType& in, pyramid_data_model_agra_SMTI_TaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_TaskBaseType_c* in, pyramid::domain_model::agra::SMTI_TaskBaseType& out);

void to_c(const pyramid::domain_model::agra::SMTI_CollectionOptionsType& in, pyramid_data_model_agra_SMTI_CollectionOptionsType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_CollectionOptionsType_c* in, pyramid::domain_model::agra::SMTI_CollectionOptionsType& out);

void to_c(const pyramid::domain_model::agra::RangeDopplerResolutionType& in, pyramid_data_model_agra_RangeDopplerResolutionType_c* out);
void from_c(const pyramid_data_model_agra_RangeDopplerResolutionType_c* in, pyramid::domain_model::agra::RangeDopplerResolutionType& out);

void to_c(const pyramid::domain_model::agra::HRR_OptionsType& in, pyramid_data_model_agra_HRR_OptionsType_c* out);
void from_c(const pyramid_data_model_agra_HRR_OptionsType_c* in, pyramid::domain_model::agra::HRR_OptionsType& out);

void to_c(const pyramid::domain_model::agra::HRR_ChipSizeType& in, pyramid_data_model_agra_HRR_ChipSizeType_c* out);
void from_c(const pyramid_data_model_agra_HRR_ChipSizeType_c* in, pyramid::domain_model::agra::HRR_ChipSizeType& out);

void to_c(const pyramid::domain_model::agra::SMTI_CollectionConstraintsType& in, pyramid_data_model_agra_SMTI_CollectionConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_CollectionConstraintsType_c* in, pyramid::domain_model::agra::SMTI_CollectionConstraintsType& out);

void to_c(const pyramid::domain_model::agra::SMTI_CollectionConstraintsQualityType& in, pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_CollectionConstraintsQualityType_c* in, pyramid::domain_model::agra::SMTI_CollectionConstraintsQualityType& out);

void to_c(const pyramid::domain_model::agra::FalseAlarmType& in, pyramid_data_model_agra_FalseAlarmType_c* out);
void from_c(const pyramid_data_model_agra_FalseAlarmType_c* in, pyramid::domain_model::agra::FalseAlarmType& out);

void to_c(const pyramid::domain_model::agra::ProductOutputCommandSMTI_Type& in, pyramid_data_model_agra_ProductOutputCommandSMTI_Type_c* out);
void from_c(const pyramid_data_model_agra_ProductOutputCommandSMTI_Type_c* in, pyramid::domain_model::agra::ProductOutputCommandSMTI_Type& out);

void to_c(const pyramid::domain_model::agra::STANAG_4607_PackingPlanPET& in, pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c* out);
void from_c(const pyramid_data_model_agra_STANAG_4607_PackingPlanPET_c* in, pyramid::domain_model::agra::STANAG_4607_PackingPlanPET& out);

void to_c(const pyramid::domain_model::agra::StrikeTaskWeaponListType& in, pyramid_data_model_agra_StrikeTaskWeaponListType_c* out);
void from_c(const pyramid_data_model_agra_StrikeTaskWeaponListType_c* in, pyramid::domain_model::agra::StrikeTaskWeaponListType& out);

void to_c(const pyramid::domain_model::agra::StrikeTaskWeaponType& in, pyramid_data_model_agra_StrikeTaskWeaponType_c* out);
void from_c(const pyramid_data_model_agra_StrikeTaskWeaponType_c* in, pyramid::domain_model::agra::StrikeTaskWeaponType& out);

void to_c(const pyramid::domain_model::agra::WeaponeeringType& in, pyramid_data_model_agra_WeaponeeringType_c* out);
void from_c(const pyramid_data_model_agra_WeaponeeringType_c* in, pyramid::domain_model::agra::WeaponeeringType& out);

void to_c(const pyramid::domain_model::agra::WeaponeeringStoreType& in, pyramid_data_model_agra_WeaponeeringStoreType_c* out);
void from_c(const pyramid_data_model_agra_WeaponeeringStoreType_c* in, pyramid::domain_model::agra::WeaponeeringStoreType& out);

void to_c(const pyramid::domain_model::agra::WeaponeeringTargetInfoType& in, pyramid_data_model_agra_WeaponeeringTargetInfoType_c* out);
void from_c(const pyramid_data_model_agra_WeaponeeringTargetInfoType_c* in, pyramid::domain_model::agra::WeaponeeringTargetInfoType& out);

void to_c(const pyramid::domain_model::agra::ApproachConditionsType& in, pyramid_data_model_agra_ApproachConditionsType_c* out);
void from_c(const pyramid_data_model_agra_ApproachConditionsType_c* in, pyramid::domain_model::agra::ApproachConditionsType& out);

void to_c(const pyramid::domain_model::agra::ApproachAngleType& in, pyramid_data_model_agra_ApproachAngleType_c* out);
void from_c(const pyramid_data_model_agra_ApproachAngleType_c* in, pyramid::domain_model::agra::ApproachAngleType& out);

void to_c(const pyramid::domain_model::agra::AzElReferenceType& in, pyramid_data_model_agra_AzElReferenceType_c* out);
void from_c(const pyramid_data_model_agra_AzElReferenceType_c* in, pyramid::domain_model::agra::AzElReferenceType& out);

void to_c(const pyramid::domain_model::agra::TargetFinalApproachType& in, pyramid_data_model_agra_TargetFinalApproachType_c* out);
void from_c(const pyramid_data_model_agra_TargetFinalApproachType_c* in, pyramid::domain_model::agra::TargetFinalApproachType& out);

void to_c(const pyramid::domain_model::agra::ImpactConditionsType& in, pyramid_data_model_agra_ImpactConditionsType_c* out);
void from_c(const pyramid_data_model_agra_ImpactConditionsType_c* in, pyramid::domain_model::agra::ImpactConditionsType& out);

void to_c(const pyramid::domain_model::agra::FuzeType& in, pyramid_data_model_agra_FuzeType_c* out);
void from_c(const pyramid_data_model_agra_FuzeType_c* in, pyramid::domain_model::agra::FuzeType& out);

void to_c(const pyramid::domain_model::agra::FuzeTriggerType& in, pyramid_data_model_agra_FuzeTriggerType_c* out);
void from_c(const pyramid_data_model_agra_FuzeTriggerType_c* in, pyramid::domain_model::agra::FuzeTriggerType& out);

void to_c(const pyramid::domain_model::agra::ImpactPointType& in, pyramid_data_model_agra_ImpactPointType_c* out);
void from_c(const pyramid_data_model_agra_ImpactPointType_c* in, pyramid::domain_model::agra::ImpactPointType& out);

void to_c(const pyramid::domain_model::agra::OffsetLocationErrorType& in, pyramid_data_model_agra_OffsetLocationErrorType_c* out);
void from_c(const pyramid_data_model_agra_OffsetLocationErrorType_c* in, pyramid::domain_model::agra::OffsetLocationErrorType& out);

void to_c(const pyramid::domain_model::agra::OffsetLocationType& in, pyramid_data_model_agra_OffsetLocationType_c* out);
void from_c(const pyramid_data_model_agra_OffsetLocationType_c* in, pyramid::domain_model::agra::OffsetLocationType& out);

void to_c(const pyramid::domain_model::agra::BodyFaceType& in, pyramid_data_model_agra_BodyFaceType_c* out);
void from_c(const pyramid_data_model_agra_BodyFaceType_c* in, pyramid::domain_model::agra::BodyFaceType& out);

void to_c(const pyramid::domain_model::agra::SystemDeploymentTaskBaseType& in, pyramid_data_model_agra_SystemDeploymentTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_SystemDeploymentTaskBaseType_c* in, pyramid::domain_model::agra::SystemDeploymentTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::TacticalOrderTaskBaseType& in, pyramid_data_model_agra_TacticalOrderTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_TacticalOrderTaskBaseType_c* in, pyramid::domain_model::agra::TacticalOrderTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::CommandResponseType& in, pyramid_data_model_agra_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_CommandResponseType_c* in, pyramid::domain_model::agra::CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::AirSampleCommandResponseType& in, pyramid_data_model_agra_AirSampleCommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_AirSampleCommandResponseType_c* in, pyramid::domain_model::agra::AirSampleCommandResponseType& out);

void to_c(const pyramid::domain_model::agra::AMTI_CommandResponseType& in, pyramid_data_model_agra_AMTI_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_AMTI_CommandResponseType_c* in, pyramid::domain_model::agra::AMTI_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::AO_CommandResponseType& in, pyramid_data_model_agra_AO_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_AO_CommandResponseType_c* in, pyramid::domain_model::agra::AO_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::COMINT_CommandResponseType& in, pyramid_data_model_agra_COMINT_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_CommandResponseType_c* in, pyramid::domain_model::agra::COMINT_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::CommRelayCommandResponseType& in, pyramid_data_model_agra_CommRelayCommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_CommRelayCommandResponseType_c* in, pyramid::domain_model::agra::CommRelayCommandResponseType& out);

void to_c(const pyramid::domain_model::agra::EA_CommandResponseType& in, pyramid_data_model_agra_EA_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_EA_CommandResponseType_c* in, pyramid::domain_model::agra::EA_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::ESM_CommandResponseType& in, pyramid_data_model_agra_ESM_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_ESM_CommandResponseType_c* in, pyramid::domain_model::agra::ESM_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::PO_CommandResponseType& in, pyramid_data_model_agra_PO_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_PO_CommandResponseType_c* in, pyramid::domain_model::agra::PO_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::PO_CollectionConstraintsSettingsType& in, pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c* out);
void from_c(const pyramid_data_model_agra_PO_CollectionConstraintsSettingsType_c* in, pyramid::domain_model::agra::PO_CollectionConstraintsSettingsType& out);

void to_c(const pyramid::domain_model::agra::PO_AngleConstraintControlsType& in, pyramid_data_model_agra_PO_AngleConstraintControlsType_c* out);
void from_c(const pyramid_data_model_agra_PO_AngleConstraintControlsType_c* in, pyramid::domain_model::agra::PO_AngleConstraintControlsType& out);

void to_c(const pyramid::domain_model::agra::PO_ConstraintControlsType& in, pyramid_data_model_agra_PO_ConstraintControlsType_c* out);
void from_c(const pyramid_data_model_agra_PO_ConstraintControlsType_c* in, pyramid::domain_model::agra::PO_ConstraintControlsType& out);

void to_c(const pyramid::domain_model::agra::PO_SlantRangeConstraintControlsType& in, pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c* out);
void from_c(const pyramid_data_model_agra_PO_SlantRangeConstraintControlsType_c* in, pyramid::domain_model::agra::PO_SlantRangeConstraintControlsType& out);

void to_c(const pyramid::domain_model::agra::PO_CollectionPatternConstraintControlsType& in, pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c* out);
void from_c(const pyramid_data_model_agra_PO_CollectionPatternConstraintControlsType_c* in, pyramid::domain_model::agra::PO_CollectionPatternConstraintControlsType& out);

void to_c(const pyramid::domain_model::agra::PO_SweepSpeedConstraintControlsType& in, pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c* out);
void from_c(const pyramid_data_model_agra_PO_SweepSpeedConstraintControlsType_c* in, pyramid::domain_model::agra::PO_SweepSpeedConstraintControlsType& out);

void to_c(const pyramid::domain_model::agra::PO_GimbalOrientationConstraintType& in, pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c* out);
void from_c(const pyramid_data_model_agra_PO_GimbalOrientationConstraintType_c* in, pyramid::domain_model::agra::PO_GimbalOrientationConstraintType& out);

void to_c(const pyramid::domain_model::agra::GimbalAxisControlType& in, pyramid_data_model_agra_GimbalAxisControlType_c* out);
void from_c(const pyramid_data_model_agra_GimbalAxisControlType_c* in, pyramid::domain_model::agra::GimbalAxisControlType& out);

void to_c(const pyramid::domain_model::agra::SAR_CommandResponseType& in, pyramid_data_model_agra_SAR_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_SAR_CommandResponseType_c* in, pyramid::domain_model::agra::SAR_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::SMTI_CommandResponseType& in, pyramid_data_model_agra_SMTI_CommandResponseType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_CommandResponseType_c* in, pyramid::domain_model::agra::SMTI_CommandResponseType& out);

void to_c(const pyramid::domain_model::agra::StrikeWeaponCommandType& in, pyramid_data_model_agra_StrikeWeaponCommandType_c* out);
void from_c(const pyramid_data_model_agra_StrikeWeaponCommandType_c* in, pyramid::domain_model::agra::StrikeWeaponCommandType& out);

void to_c(const pyramid::domain_model::agra::RequirementConstraintsType& in, pyramid_data_model_agra_RequirementConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_RequirementConstraintsType_c* in, pyramid::domain_model::agra::RequirementConstraintsType& out);

void to_c(const pyramid::domain_model::agra::RequirementAllocationParametersType& in, pyramid_data_model_agra_RequirementAllocationParametersType_c* out);
void from_c(const pyramid_data_model_agra_RequirementAllocationParametersType_c* in, pyramid::domain_model::agra::RequirementAllocationParametersType& out);

void to_c(const pyramid::domain_model::agra::RequirementAllocationConstraintType& in, pyramid_data_model_agra_RequirementAllocationConstraintType_c* out);
void from_c(const pyramid_data_model_agra_RequirementAllocationConstraintType_c* in, pyramid::domain_model::agra::RequirementAllocationConstraintType& out);

void to_c(const pyramid::domain_model::agra::RequirementKinematicConstraintsType& in, pyramid_data_model_agra_RequirementKinematicConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_RequirementKinematicConstraintsType_c* in, pyramid::domain_model::agra::RequirementKinematicConstraintsType& out);

void to_c(const pyramid::domain_model::agra::CapabilityCommandTemporalConstraintsType& in, pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_CapabilityCommandTemporalConstraintsType_c* in, pyramid::domain_model::agra::CapabilityCommandTemporalConstraintsType& out);

void to_c(const pyramid::domain_model::agra::RequirementGenerationDependencyType& in, pyramid_data_model_agra_RequirementGenerationDependencyType_c* out);
void from_c(const pyramid_data_model_agra_RequirementGenerationDependencyType_c* in, pyramid::domain_model::agra::RequirementGenerationDependencyType& out);

void to_c(const pyramid::domain_model::agra::MA_TaskMT& in, pyramid_data_model_agra_MA_TaskMT_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskMT_c* in, pyramid::domain_model::agra::MA_TaskMT& out);

void to_c(const pyramid::domain_model::agra::MA_TaskMDT& in, pyramid_data_model_agra_MA_TaskMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskMDT_c* in, pyramid::domain_model::agra::MA_TaskMDT& out);

void to_c(const pyramid::domain_model::agra::MA_TaskType& in, pyramid_data_model_agra_MA_TaskType_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskType_c* in, pyramid::domain_model::agra::MA_TaskType& out);

void to_c(const pyramid::domain_model::agra::AirSampleTaskType& in, pyramid_data_model_agra_AirSampleTaskType_c* out);
void from_c(const pyramid_data_model_agra_AirSampleTaskType_c* in, pyramid::domain_model::agra::AirSampleTaskType& out);

void to_c(const pyramid::domain_model::agra::AMTI_TaskType& in, pyramid_data_model_agra_AMTI_TaskType_c* out);
void from_c(const pyramid_data_model_agra_AMTI_TaskType_c* in, pyramid::domain_model::agra::AMTI_TaskType& out);

void to_c(const pyramid::domain_model::agra::AMTI_TargetType& in, pyramid_data_model_agra_AMTI_TargetType_c* out);
void from_c(const pyramid_data_model_agra_AMTI_TargetType_c* in, pyramid::domain_model::agra::AMTI_TargetType& out);

void to_c(const pyramid::domain_model::agra::AO_TaskType& in, pyramid_data_model_agra_AO_TaskType_c* out);
void from_c(const pyramid_data_model_agra_AO_TaskType_c* in, pyramid::domain_model::agra::AO_TaskType& out);

void to_c(const pyramid::domain_model::agra::MA_CAP_TaskType& in, pyramid_data_model_agra_MA_CAP_TaskType_c* out);
void from_c(const pyramid_data_model_agra_MA_CAP_TaskType_c* in, pyramid::domain_model::agra::MA_CAP_TaskType& out);

void to_c(const pyramid::domain_model::agra::MA_OrbitType& in, pyramid_data_model_agra_MA_OrbitType_c* out);
void from_c(const pyramid_data_model_agra_MA_OrbitType_c* in, pyramid::domain_model::agra::MA_OrbitType& out);

void to_c(const pyramid::domain_model::agra::MA_OrbitShapeType& in, pyramid_data_model_agra_MA_OrbitShapeType_c* out);
void from_c(const pyramid_data_model_agra_MA_OrbitShapeType_c* in, pyramid::domain_model::agra::MA_OrbitShapeType& out);

void to_c(const pyramid::domain_model::agra::MA_FixOrbitType& in, pyramid_data_model_agra_MA_FixOrbitType_c* out);
void from_c(const pyramid_data_model_agra_MA_FixOrbitType_c* in, pyramid::domain_model::agra::MA_FixOrbitType& out);

void to_c(const pyramid::domain_model::agra::MA_DirectionChoiceType& in, pyramid_data_model_agra_MA_DirectionChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_DirectionChoiceType_c* in, pyramid::domain_model::agra::MA_DirectionChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_DirectionReferenceType& in, pyramid_data_model_agra_MA_DirectionReferenceType_c* out);
void from_c(const pyramid_data_model_agra_MA_DirectionReferenceType_c* in, pyramid::domain_model::agra::MA_DirectionReferenceType& out);

void to_c(const pyramid::domain_model::agra::IntervalChoiceType& in, pyramid_data_model_agra_IntervalChoiceType_c* out);
void from_c(const pyramid_data_model_agra_IntervalChoiceType_c* in, pyramid::domain_model::agra::IntervalChoiceType& out);

void to_c(const pyramid::domain_model::agra::TurnGeometryChoiceType& in, pyramid_data_model_agra_TurnGeometryChoiceType_c* out);
void from_c(const pyramid_data_model_agra_TurnGeometryChoiceType_c* in, pyramid::domain_model::agra::TurnGeometryChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_CircleOrbitType& in, pyramid_data_model_agra_MA_CircleOrbitType_c* out);
void from_c(const pyramid_data_model_agra_MA_CircleOrbitType_c* in, pyramid::domain_model::agra::MA_CircleOrbitType& out);

void to_c(const pyramid::domain_model::agra::CircleType& in, pyramid_data_model_agra_CircleType_c* out);
void from_c(const pyramid_data_model_agra_CircleType_c* in, pyramid::domain_model::agra::CircleType& out);

void to_c(const pyramid::domain_model::agra::OrbitDurationType& in, pyramid_data_model_agra_OrbitDurationType_c* out);
void from_c(const pyramid_data_model_agra_OrbitDurationType_c* in, pyramid::domain_model::agra::OrbitDurationType& out);

void to_c(const pyramid::domain_model::agra::MA_SynchronizationChoiceType& in, pyramid_data_model_agra_MA_SynchronizationChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_SynchronizationChoiceType_c* in, pyramid::domain_model::agra::MA_SynchronizationChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_CAPRelativePositionType& in, pyramid_data_model_agra_MA_CAPRelativePositionType_c* out);
void from_c(const pyramid_data_model_agra_MA_CAPRelativePositionType_c* in, pyramid::domain_model::agra::MA_CAPRelativePositionType& out);

void to_c(const pyramid::domain_model::agra::MA_CAPReferencePointType& in, pyramid_data_model_agra_MA_CAPReferencePointType_c* out);
void from_c(const pyramid_data_model_agra_MA_CAPReferencePointType_c* in, pyramid::domain_model::agra::MA_CAPReferencePointType& out);

void to_c(const pyramid::domain_model::agra::MA_CAPOffsetType& in, pyramid_data_model_agra_MA_CAPOffsetType_c* out);
void from_c(const pyramid_data_model_agra_MA_CAPOffsetType_c* in, pyramid::domain_model::agra::MA_CAPOffsetType& out);

void to_c(const pyramid::domain_model::agra::MA_SynchronizationChoiceType_RelativePositioning_List& in, pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c* out);
void from_c(const pyramid_data_model_agra_MA_SynchronizationChoiceType_RelativePositioning_List_c* in, pyramid::domain_model::agra::MA_SynchronizationChoiceType_RelativePositioning_List& out);

void to_c(const pyramid::domain_model::agra::PathSegmentSpeedType& in, pyramid_data_model_agra_PathSegmentSpeedType_c* out);
void from_c(const pyramid_data_model_agra_PathSegmentSpeedType_c* in, pyramid::domain_model::agra::PathSegmentSpeedType& out);

void to_c(const pyramid::domain_model::agra::PathSegmentSpeedChoiceType& in, pyramid_data_model_agra_PathSegmentSpeedChoiceType_c* out);
void from_c(const pyramid_data_model_agra_PathSegmentSpeedChoiceType_c* in, pyramid::domain_model::agra::PathSegmentSpeedChoiceType& out);

void to_c(const pyramid::domain_model::agra::PathSegmentSpeedValueType& in, pyramid_data_model_agra_PathSegmentSpeedValueType_c* out);
void from_c(const pyramid_data_model_agra_PathSegmentSpeedValueType_c* in, pyramid::domain_model::agra::PathSegmentSpeedValueType& out);

void to_c(const pyramid::domain_model::agra::CargoDeliveryTaskType& in, pyramid_data_model_agra_CargoDeliveryTaskType_c* out);
void from_c(const pyramid_data_model_agra_CargoDeliveryTaskType_c* in, pyramid::domain_model::agra::CargoDeliveryTaskType& out);

void to_c(const pyramid::domain_model::agra::CargoTransitionType& in, pyramid_data_model_agra_CargoTransitionType_c* out);
void from_c(const pyramid_data_model_agra_CargoTransitionType_c* in, pyramid::domain_model::agra::CargoTransitionType& out);

void to_c(const pyramid::domain_model::agra::CargoID_Type& in, pyramid_data_model_agra_CargoID_Type_c* out);
void from_c(const pyramid_data_model_agra_CargoID_Type_c* in, pyramid::domain_model::agra::CargoID_Type& out);

void to_c(const pyramid::domain_model::agra::SectorType& in, pyramid_data_model_agra_SectorType_c* out);
void from_c(const pyramid_data_model_agra_SectorType_c* in, pyramid::domain_model::agra::SectorType& out);

void to_c(const pyramid::domain_model::agra::CargoDeliveryTaskType_Dropoff_List& in, pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c* out);
void from_c(const pyramid_data_model_agra_CargoDeliveryTaskType_Dropoff_List_c* in, pyramid::domain_model::agra::CargoDeliveryTaskType_Dropoff_List& out);

void to_c(const pyramid::domain_model::agra::COMINT_TaskType& in, pyramid_data_model_agra_COMINT_TaskType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_TaskType_c* in, pyramid::domain_model::agra::COMINT_TaskType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityChoiceType& in, pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityChoiceType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityChoiceType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityAcquisitionType& in, pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityAcquisitionType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityAcquisitionType& out);

void to_c(const pyramid::domain_model::agra::COMINT_AcquisitionTargetType& in, pyramid_data_model_agra_COMINT_AcquisitionTargetType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_AcquisitionTargetType_c* in, pyramid::domain_model::agra::COMINT_AcquisitionTargetType& out);

void to_c(const pyramid::domain_model::agra::COMINT_TargetType& in, pyramid_data_model_agra_COMINT_TargetType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_TargetType_c* in, pyramid::domain_model::agra::COMINT_TargetType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityTargetLocationDataType& in, pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityTargetLocationDataType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityTargetLocationDataType& out);

void to_c(const pyramid::domain_model::agra::NED_ConeType& in, pyramid_data_model_agra_NED_ConeType_c* out);
void from_c(const pyramid_data_model_agra_NED_ConeType_c* in, pyramid::domain_model::agra::NED_ConeType& out);

void to_c(const pyramid::domain_model::agra::NED_LOS_Type& in, pyramid_data_model_agra_NED_LOS_Type_c* out);
void from_c(const pyramid_data_model_agra_NED_LOS_Type_c* in, pyramid::domain_model::agra::NED_LOS_Type& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityIdentificationType& in, pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityIdentificationType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityIdentificationType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityGeolocationType& in, pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityGeolocationType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityGeolocationType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityMeasurementType& in, pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityMeasurementType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityMeasurementType& out);

void to_c(const pyramid::domain_model::agra::COMINT_SubcapabilityDataCollectType& in, pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_SubcapabilityDataCollectType_c* in, pyramid::domain_model::agra::COMINT_SubcapabilityDataCollectType& out);

void to_c(const pyramid::domain_model::agra::COMINT_DataCollectCommandType& in, pyramid_data_model_agra_COMINT_DataCollectCommandType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_DataCollectCommandType_c* in, pyramid::domain_model::agra::COMINT_DataCollectCommandType& out);

void to_c(const pyramid::domain_model::agra::CollectionFrequencyType& in, pyramid_data_model_agra_CollectionFrequencyType_c* out);
void from_c(const pyramid_data_model_agra_CollectionFrequencyType_c* in, pyramid::domain_model::agra::CollectionFrequencyType& out);

void to_c(const pyramid::domain_model::agra::COMINT_InteractiveType& in, pyramid_data_model_agra_COMINT_InteractiveType_c* out);
void from_c(const pyramid_data_model_agra_COMINT_InteractiveType_c* in, pyramid::domain_model::agra::COMINT_InteractiveType& out);

void to_c(const pyramid::domain_model::agra::CommRelayTaskType& in, pyramid_data_model_agra_CommRelayTaskType_c* out);
void from_c(const pyramid_data_model_agra_CommRelayTaskType_c* in, pyramid::domain_model::agra::CommRelayTaskType& out);

void to_c(const pyramid::domain_model::agra::LocationType& in, pyramid_data_model_agra_LocationType_c* out);
void from_c(const pyramid_data_model_agra_LocationType_c* in, pyramid::domain_model::agra::LocationType& out);

void to_c(const pyramid::domain_model::agra::PathType& in, pyramid_data_model_agra_PathType_c* out);
void from_c(const pyramid_data_model_agra_PathType_c* in, pyramid::domain_model::agra::PathType& out);

void to_c(const pyramid::domain_model::agra::LoiterType& in, pyramid_data_model_agra_LoiterType_c* out);
void from_c(const pyramid_data_model_agra_LoiterType_c* in, pyramid::domain_model::agra::LoiterType& out);

void to_c(const pyramid::domain_model::agra::OrbitType& in, pyramid_data_model_agra_OrbitType_c* out);
void from_c(const pyramid_data_model_agra_OrbitType_c* in, pyramid::domain_model::agra::OrbitType& out);

void to_c(const pyramid::domain_model::agra::HoverType& in, pyramid_data_model_agra_HoverType_c* out);
void from_c(const pyramid_data_model_agra_HoverType_c* in, pyramid::domain_model::agra::HoverType& out);

void to_c(const pyramid::domain_model::agra::PointChoice3D_Type& in, pyramid_data_model_agra_PointChoice3D_Type_c* out);
void from_c(const pyramid_data_model_agra_PointChoice3D_Type_c* in, pyramid::domain_model::agra::PointChoice3D_Type& out);

void to_c(const pyramid::domain_model::agra::Point3D_RelativeType& in, pyramid_data_model_agra_Point3D_RelativeType_c* out);
void from_c(const pyramid_data_model_agra_Point3D_RelativeType_c* in, pyramid::domain_model::agra::Point3D_RelativeType& out);

void to_c(const pyramid::domain_model::agra::CounterSpaceTaskType& in, pyramid_data_model_agra_CounterSpaceTaskType_c* out);
void from_c(const pyramid_data_model_agra_CounterSpaceTaskType_c* in, pyramid::domain_model::agra::CounterSpaceTaskType& out);

void to_c(const pyramid::domain_model::agra::CS_EngagementDataType& in, pyramid_data_model_agra_CS_EngagementDataType_c* out);
void from_c(const pyramid_data_model_agra_CS_EngagementDataType_c* in, pyramid::domain_model::agra::CS_EngagementDataType& out);

void to_c(const pyramid::domain_model::agra::CS_DetailDataType& in, pyramid_data_model_agra_CS_DetailDataType_c* out);
void from_c(const pyramid_data_model_agra_CS_DetailDataType_c* in, pyramid::domain_model::agra::CS_DetailDataType& out);

void to_c(const pyramid::domain_model::agra::CS_SubDetailDataType& in, pyramid_data_model_agra_CS_SubDetailDataType_c* out);
void from_c(const pyramid_data_model_agra_CS_SubDetailDataType_c* in, pyramid::domain_model::agra::CS_SubDetailDataType& out);

void to_c(const pyramid::domain_model::agra::CS_SignalType& in, pyramid_data_model_agra_CS_SignalType_c* out);
void from_c(const pyramid_data_model_agra_CS_SignalType_c* in, pyramid::domain_model::agra::CS_SignalType& out);

void to_c(const pyramid::domain_model::agra::MA_EscortTaskType& in, pyramid_data_model_agra_MA_EscortTaskType_c* out);
void from_c(const pyramid_data_model_agra_MA_EscortTaskType_c* in, pyramid::domain_model::agra::MA_EscortTaskType& out);

void to_c(const pyramid::domain_model::agra::MA_EscortAssetType& in, pyramid_data_model_agra_MA_EscortAssetType_c* out);
void from_c(const pyramid_data_model_agra_MA_EscortAssetType_c* in, pyramid::domain_model::agra::MA_EscortAssetType& out);

void to_c(const pyramid::domain_model::agra::MA_EscortReferenceType& in, pyramid_data_model_agra_MA_EscortReferenceType_c* out);
void from_c(const pyramid_data_model_agra_MA_EscortReferenceType_c* in, pyramid::domain_model::agra::MA_EscortReferenceType& out);

void to_c(const pyramid::domain_model::agra::MA_EscortFollowPathType& in, pyramid_data_model_agra_MA_EscortFollowPathType_c* out);
void from_c(const pyramid_data_model_agra_MA_EscortFollowPathType_c* in, pyramid::domain_model::agra::MA_EscortFollowPathType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskType& in, pyramid_data_model_agra_EA_TaskType_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskType_c* in, pyramid::domain_model::agra::EA_TaskType& out);

void to_c(const pyramid::domain_model::agra::EA_TaskRouteRequirementsType& in, pyramid_data_model_agra_EA_TaskRouteRequirementsType_c* out);
void from_c(const pyramid_data_model_agra_EA_TaskRouteRequirementsType_c* in, pyramid::domain_model::agra::EA_TaskRouteRequirementsType& out);

void to_c(const pyramid::domain_model::agra::ZoneChoiceType& in, pyramid_data_model_agra_ZoneChoiceType_c* out);
void from_c(const pyramid_data_model_agra_ZoneChoiceType_c* in, pyramid::domain_model::agra::ZoneChoiceType& out);

void to_c(const pyramid::domain_model::agra::VolumeChoiceType& in, pyramid_data_model_agra_VolumeChoiceType_c* out);
void from_c(const pyramid_data_model_agra_VolumeChoiceType_c* in, pyramid::domain_model::agra::VolumeChoiceType& out);

void to_c(const pyramid::domain_model::agra::ESM_TaskType& in, pyramid_data_model_agra_ESM_TaskType_c* out);
void from_c(const pyramid_data_model_agra_ESM_TaskType_c* in, pyramid::domain_model::agra::ESM_TaskType& out);

void to_c(const pyramid::domain_model::agra::SubCapabilityDetailsType& in, pyramid_data_model_agra_SubCapabilityDetailsType_c* out);
void from_c(const pyramid_data_model_agra_SubCapabilityDetailsType_c* in, pyramid::domain_model::agra::SubCapabilityDetailsType& out);

void to_c(const pyramid::domain_model::agra::SelectAntennaType& in, pyramid_data_model_agra_SelectAntennaType_c* out);
void from_c(const pyramid_data_model_agra_SelectAntennaType_c* in, pyramid::domain_model::agra::SelectAntennaType& out);

void to_c(const pyramid::domain_model::agra::SupportCapabilityID_Type& in, pyramid_data_model_agra_SupportCapabilityID_Type_c* out);
void from_c(const pyramid_data_model_agra_SupportCapabilityID_Type_c* in, pyramid::domain_model::agra::SupportCapabilityID_Type& out);

void to_c(const pyramid::domain_model::agra::AntennaResourceChoiceType& in, pyramid_data_model_agra_AntennaResourceChoiceType_c* out);
void from_c(const pyramid_data_model_agra_AntennaResourceChoiceType_c* in, pyramid::domain_model::agra::AntennaResourceChoiceType& out);

void to_c(const pyramid::domain_model::agra::AntennaResourceID_Type& in, pyramid_data_model_agra_AntennaResourceID_Type_c* out);
void from_c(const pyramid_data_model_agra_AntennaResourceID_Type_c* in, pyramid::domain_model::agra::AntennaResourceID_Type& out);

void to_c(const pyramid::domain_model::agra::ESM_TargetType& in, pyramid_data_model_agra_ESM_TargetType_c* out);
void from_c(const pyramid_data_model_agra_ESM_TargetType_c* in, pyramid::domain_model::agra::ESM_TargetType& out);

void to_c(const pyramid::domain_model::agra::ESM_LocationType& in, pyramid_data_model_agra_ESM_LocationType_c* out);
void from_c(const pyramid_data_model_agra_ESM_LocationType_c* in, pyramid::domain_model::agra::ESM_LocationType& out);

void to_c(const pyramid::domain_model::agra::ESM_SubcapabilityTargetLocationDataType& in, pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c* out);
void from_c(const pyramid_data_model_agra_ESM_SubcapabilityTargetLocationDataType_c* in, pyramid::domain_model::agra::ESM_SubcapabilityTargetLocationDataType& out);

void to_c(const pyramid::domain_model::agra::PulseDataCollectCommandType& in, pyramid_data_model_agra_PulseDataCollectCommandType_c* out);
void from_c(const pyramid_data_model_agra_PulseDataCollectCommandType_c* in, pyramid::domain_model::agra::PulseDataCollectCommandType& out);

void to_c(const pyramid::domain_model::agra::ESM_SubcapabilityGeolocationType& in, pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c* out);
void from_c(const pyramid_data_model_agra_ESM_SubcapabilityGeolocationType_c* in, pyramid::domain_model::agra::ESM_SubcapabilityGeolocationType& out);

void to_c(const pyramid::domain_model::agra::MA_FlightTaskType& in, pyramid_data_model_agra_MA_FlightTaskType_c* out);
void from_c(const pyramid_data_model_agra_MA_FlightTaskType_c* in, pyramid::domain_model::agra::MA_FlightTaskType& out);

void to_c(const pyramid::domain_model::agra::MA_FlightTaskBaseType& in, pyramid_data_model_agra_MA_FlightTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_MA_FlightTaskBaseType_c* in, pyramid::domain_model::agra::MA_FlightTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::MA_LoiterType& in, pyramid_data_model_agra_MA_LoiterType_c* out);
void from_c(const pyramid_data_model_agra_MA_LoiterType_c* in, pyramid::domain_model::agra::MA_LoiterType& out);

void to_c(const pyramid::domain_model::agra::MA_HoldType& in, pyramid_data_model_agra_MA_HoldType_c* out);
void from_c(const pyramid_data_model_agra_MA_HoldType_c* in, pyramid::domain_model::agra::MA_HoldType& out);

void to_c(const pyramid::domain_model::agra::MA_HoldLegSpecificationType& in, pyramid_data_model_agra_MA_HoldLegSpecificationType_c* out);
void from_c(const pyramid_data_model_agra_MA_HoldLegSpecificationType_c* in, pyramid::domain_model::agra::MA_HoldLegSpecificationType& out);

void to_c(const pyramid::domain_model::agra::MA_HoldTurnSpecificationType& in, pyramid_data_model_agra_MA_HoldTurnSpecificationType_c* out);
void from_c(const pyramid_data_model_agra_MA_HoldTurnSpecificationType_c* in, pyramid::domain_model::agra::MA_HoldTurnSpecificationType& out);

void to_c(const pyramid::domain_model::agra::MA_OrbitDurationType& in, pyramid_data_model_agra_MA_OrbitDurationType_c* out);
void from_c(const pyramid_data_model_agra_MA_OrbitDurationType_c* in, pyramid::domain_model::agra::MA_OrbitDurationType& out);

void to_c(const pyramid::domain_model::agra::MustFlyType& in, pyramid_data_model_agra_MustFlyType_c* out);
void from_c(const pyramid_data_model_agra_MustFlyType_c* in, pyramid::domain_model::agra::MustFlyType& out);

void to_c(const pyramid::domain_model::agra::MustFlyLocationType& in, pyramid_data_model_agra_MustFlyLocationType_c* out);
void from_c(const pyramid_data_model_agra_MustFlyLocationType_c* in, pyramid::domain_model::agra::MustFlyLocationType& out);

void to_c(const pyramid::domain_model::agra::MA_FormationType& in, pyramid_data_model_agra_MA_FormationType_c* out);
void from_c(const pyramid_data_model_agra_MA_FormationType_c* in, pyramid::domain_model::agra::MA_FormationType& out);

void to_c(const pyramid::domain_model::agra::MA_FormationAnchorType& in, pyramid_data_model_agra_MA_FormationAnchorType_c* out);
void from_c(const pyramid_data_model_agra_MA_FormationAnchorType_c* in, pyramid::domain_model::agra::MA_FormationAnchorType& out);

void to_c(const pyramid::domain_model::agra::MA_AltitudeStackedMarshallType& in, pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c* out);
void from_c(const pyramid_data_model_agra_MA_AltitudeStackedMarshallType_c* in, pyramid::domain_model::agra::MA_AltitudeStackedMarshallType& out);

void to_c(const pyramid::domain_model::agra::MA_FlightControlModesChoiceType& in, pyramid_data_model_agra_MA_FlightControlModesChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_FlightControlModesChoiceType_c* in, pyramid::domain_model::agra::MA_FlightControlModesChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_HSA_CSA_Type& in, pyramid_data_model_agra_MA_HSA_CSA_Type_c* out);
void from_c(const pyramid_data_model_agra_MA_HSA_CSA_Type_c* in, pyramid::domain_model::agra::MA_HSA_CSA_Type& out);

void to_c(const pyramid::domain_model::agra::MA_WaypointFollowingType& in, pyramid_data_model_agra_MA_WaypointFollowingType_c* out);
void from_c(const pyramid_data_model_agra_MA_WaypointFollowingType_c* in, pyramid::domain_model::agra::MA_WaypointFollowingType& out);

void to_c(const pyramid::domain_model::agra::MA_RouteType& in, pyramid_data_model_agra_MA_RouteType_c* out);
void from_c(const pyramid_data_model_agra_MA_RouteType_c* in, pyramid::domain_model::agra::MA_RouteType& out);

void to_c(const pyramid::domain_model::agra::MA_RoutePathType& in, pyramid_data_model_agra_MA_RoutePathType_c* out);
void from_c(const pyramid_data_model_agra_MA_RoutePathType_c* in, pyramid::domain_model::agra::MA_RoutePathType& out);

void to_c(const pyramid::domain_model::agra::MA_PathSegmentType& in, pyramid_data_model_agra_MA_PathSegmentType_c* out);
void from_c(const pyramid_data_model_agra_MA_PathSegmentType_c* in, pyramid::domain_model::agra::MA_PathSegmentType& out);

void to_c(const pyramid::domain_model::agra::MA_EndPointType& in, pyramid_data_model_agra_MA_EndPointType_c* out);
void from_c(const pyramid_data_model_agra_MA_EndPointType_c* in, pyramid::domain_model::agra::MA_EndPointType& out);

void to_c(const pyramid::domain_model::agra::TurnPointType& in, pyramid_data_model_agra_TurnPointType_c* out);
void from_c(const pyramid_data_model_agra_TurnPointType_c* in, pyramid::domain_model::agra::TurnPointType& out);

void to_c(const pyramid::domain_model::agra::MA_LoiterPointType& in, pyramid_data_model_agra_MA_LoiterPointType_c* out);
void from_c(const pyramid_data_model_agra_MA_LoiterPointType_c* in, pyramid::domain_model::agra::MA_LoiterPointType& out);

void to_c(const pyramid::domain_model::agra::CivilPathTerminatorType& in, pyramid_data_model_agra_CivilPathTerminatorType_c* out);
void from_c(const pyramid_data_model_agra_CivilPathTerminatorType_c* in, pyramid::domain_model::agra::CivilPathTerminatorType& out);

void to_c(const pyramid::domain_model::agra::CF_CourseToFixType& in, pyramid_data_model_agra_CF_CourseToFixType_c* out);
void from_c(const pyramid_data_model_agra_CF_CourseToFixType_c* in, pyramid::domain_model::agra::CF_CourseToFixType& out);

void to_c(const pyramid::domain_model::agra::RF_RadiusToFixType& in, pyramid_data_model_agra_RF_RadiusToFixType_c* out);
void from_c(const pyramid_data_model_agra_RF_RadiusToFixType_c* in, pyramid::domain_model::agra::RF_RadiusToFixType& out);

void to_c(const pyramid::domain_model::agra::ClimbType& in, pyramid_data_model_agra_ClimbType_c* out);
void from_c(const pyramid_data_model_agra_ClimbType_c* in, pyramid::domain_model::agra::ClimbType& out);

void to_c(const pyramid::domain_model::agra::NextPathSegmentType& in, pyramid_data_model_agra_NextPathSegmentType_c* out);
void from_c(const pyramid_data_model_agra_NextPathSegmentType_c* in, pyramid::domain_model::agra::NextPathSegmentType& out);

void to_c(const pyramid::domain_model::agra::ConditionalPathSegmentType& in, pyramid_data_model_agra_ConditionalPathSegmentType_c* out);
void from_c(const pyramid_data_model_agra_ConditionalPathSegmentType_c* in, pyramid::domain_model::agra::ConditionalPathSegmentType& out);

void to_c(const pyramid::domain_model::agra::PathSegmentConditionType& in, pyramid_data_model_agra_PathSegmentConditionType_c* out);
void from_c(const pyramid_data_model_agra_PathSegmentConditionType_c* in, pyramid::domain_model::agra::PathSegmentConditionType& out);

void to_c(const pyramid::domain_model::agra::SegmentCaptureType& in, pyramid_data_model_agra_SegmentCaptureType_c* out);
void from_c(const pyramid_data_model_agra_SegmentCaptureType_c* in, pyramid::domain_model::agra::SegmentCaptureType& out);

void to_c(const pyramid::domain_model::agra::EnduranceRemainingType& in, pyramid_data_model_agra_EnduranceRemainingType_c* out);
void from_c(const pyramid_data_model_agra_EnduranceRemainingType_c* in, pyramid::domain_model::agra::EnduranceRemainingType& out);

void to_c(const pyramid::domain_model::agra::AirfieldID_Type& in, pyramid_data_model_agra_AirfieldID_Type_c* out);
void from_c(const pyramid_data_model_agra_AirfieldID_Type_c* in, pyramid::domain_model::agra::AirfieldID_Type& out);

void to_c(const pyramid::domain_model::agra::RunwayID_Type& in, pyramid_data_model_agra_RunwayID_Type_c* out);
void from_c(const pyramid_data_model_agra_RunwayID_Type_c* in, pyramid::domain_model::agra::RunwayID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_CurveControlType& in, pyramid_data_model_agra_MA_CurveControlType_c* out);
void from_c(const pyramid_data_model_agra_MA_CurveControlType_c* in, pyramid::domain_model::agra::MA_CurveControlType& out);

void to_c(const pyramid::domain_model::agra::MA_NURBS_PointType& in, pyramid_data_model_agra_MA_NURBS_PointType_c* out);
void from_c(const pyramid_data_model_agra_MA_NURBS_PointType_c* in, pyramid::domain_model::agra::MA_NURBS_PointType& out);

void to_c(const pyramid::domain_model::agra::MA_NURBS_ControlPointType& in, pyramid_data_model_agra_MA_NURBS_ControlPointType_c* out);
void from_c(const pyramid_data_model_agra_MA_NURBS_ControlPointType_c* in, pyramid::domain_model::agra::MA_NURBS_ControlPointType& out);

void to_c(const pyramid::domain_model::agra::MA_CurveTraversingType& in, pyramid_data_model_agra_MA_CurveTraversingType_c* out);
void from_c(const pyramid_data_model_agra_MA_CurveTraversingType_c* in, pyramid::domain_model::agra::MA_CurveTraversingType& out);

void to_c(const pyramid::domain_model::agra::MA_LaunchType& in, pyramid_data_model_agra_MA_LaunchType_c* out);
void from_c(const pyramid_data_model_agra_MA_LaunchType_c* in, pyramid::domain_model::agra::MA_LaunchType& out);

void to_c(const pyramid::domain_model::agra::MA_CarrierLaunchType& in, pyramid_data_model_agra_MA_CarrierLaunchType_c* out);
void from_c(const pyramid_data_model_agra_MA_CarrierLaunchType_c* in, pyramid::domain_model::agra::MA_CarrierLaunchType& out);

void to_c(const pyramid::domain_model::agra::MA_CatapultID_Type& in, pyramid_data_model_agra_MA_CatapultID_Type_c* out);
void from_c(const pyramid_data_model_agra_MA_CatapultID_Type_c* in, pyramid::domain_model::agra::MA_CatapultID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_AirfieldTakeoffType& in, pyramid_data_model_agra_MA_AirfieldTakeoffType_c* out);
void from_c(const pyramid_data_model_agra_MA_AirfieldTakeoffType_c* in, pyramid::domain_model::agra::MA_AirfieldTakeoffType& out);

void to_c(const pyramid::domain_model::agra::MA_RecoveryType& in, pyramid_data_model_agra_MA_RecoveryType_c* out);
void from_c(const pyramid_data_model_agra_MA_RecoveryType_c* in, pyramid::domain_model::agra::MA_RecoveryType& out);

void to_c(const pyramid::domain_model::agra::MA_CarrierRecoveryChoiceType& in, pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c* out);
void from_c(const pyramid_data_model_agra_MA_CarrierRecoveryChoiceType_c* in, pyramid::domain_model::agra::MA_CarrierRecoveryChoiceType& out);

void to_c(const pyramid::domain_model::agra::MA_CarrierRecoveryType& in, pyramid_data_model_agra_MA_CarrierRecoveryType_c* out);
void from_c(const pyramid_data_model_agra_MA_CarrierRecoveryType_c* in, pyramid::domain_model::agra::MA_CarrierRecoveryType& out);

void to_c(const pyramid::domain_model::agra::MA_DeltaType& in, pyramid_data_model_agra_MA_DeltaType_c* out);
void from_c(const pyramid_data_model_agra_MA_DeltaType_c* in, pyramid::domain_model::agra::MA_DeltaType& out);

void to_c(const pyramid::domain_model::agra::MA_AirfieldLandType& in, pyramid_data_model_agra_MA_AirfieldLandType_c* out);
void from_c(const pyramid_data_model_agra_MA_AirfieldLandType_c* in, pyramid::domain_model::agra::MA_AirfieldLandType& out);

void to_c(const pyramid::domain_model::agra::MA_JettisonTaskType& in, pyramid_data_model_agra_MA_JettisonTaskType_c* out);
void from_c(const pyramid_data_model_agra_MA_JettisonTaskType_c* in, pyramid::domain_model::agra::MA_JettisonTaskType& out);

void to_c(const pyramid::domain_model::agra::MA_JettisonStoreSelectionType& in, pyramid_data_model_agra_MA_JettisonStoreSelectionType_c* out);
void from_c(const pyramid_data_model_agra_MA_JettisonStoreSelectionType_c* in, pyramid::domain_model::agra::MA_JettisonStoreSelectionType& out);

void to_c(const pyramid::domain_model::agra::MA_JettisonStoreSelectionType_CapabilityID_List& in, pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c* out);
void from_c(const pyramid_data_model_agra_MA_JettisonStoreSelectionType_CapabilityID_List_c* in, pyramid::domain_model::agra::MA_JettisonStoreSelectionType_CapabilityID_List& out);

void to_c(const pyramid::domain_model::agra::OrbitChangeTaskType& in, pyramid_data_model_agra_OrbitChangeTaskType_c* out);
void from_c(const pyramid_data_model_agra_OrbitChangeTaskType_c* in, pyramid::domain_model::agra::OrbitChangeTaskType& out);

void to_c(const pyramid::domain_model::agra::OrbitChangeChoiceType& in, pyramid_data_model_agra_OrbitChangeChoiceType_c* out);
void from_c(const pyramid_data_model_agra_OrbitChangeChoiceType_c* in, pyramid::domain_model::agra::OrbitChangeChoiceType& out);

void to_c(const pyramid::domain_model::agra::COE_OrbitType& in, pyramid_data_model_agra_COE_OrbitType_c* out);
void from_c(const pyramid_data_model_agra_COE_OrbitType_c* in, pyramid::domain_model::agra::COE_OrbitType& out);

void to_c(const pyramid::domain_model::agra::OrbitalVolumeType& in, pyramid_data_model_agra_OrbitalVolumeType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalVolumeType_c* in, pyramid::domain_model::agra::OrbitalVolumeType& out);

void to_c(const pyramid::domain_model::agra::RSO_ApproachType& in, pyramid_data_model_agra_RSO_ApproachType_c* out);
void from_c(const pyramid_data_model_agra_RSO_ApproachType_c* in, pyramid::domain_model::agra::RSO_ApproachType& out);

void to_c(const pyramid::domain_model::agra::ProximityOperationsType& in, pyramid_data_model_agra_ProximityOperationsType_c* out);
void from_c(const pyramid_data_model_agra_ProximityOperationsType_c* in, pyramid::domain_model::agra::ProximityOperationsType& out);

void to_c(const pyramid::domain_model::agra::ProximityOrbitChoiceType& in, pyramid_data_model_agra_ProximityOrbitChoiceType_c* out);
void from_c(const pyramid_data_model_agra_ProximityOrbitChoiceType_c* in, pyramid::domain_model::agra::ProximityOrbitChoiceType& out);

void to_c(const pyramid::domain_model::agra::RaceTrackOrbitType& in, pyramid_data_model_agra_RaceTrackOrbitType_c* out);
void from_c(const pyramid_data_model_agra_RaceTrackOrbitType_c* in, pyramid::domain_model::agra::RaceTrackOrbitType& out);

void to_c(const pyramid::domain_model::agra::DisposalOrbitType& in, pyramid_data_model_agra_DisposalOrbitType_c* out);
void from_c(const pyramid_data_model_agra_DisposalOrbitType_c* in, pyramid::domain_model::agra::DisposalOrbitType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceTaskType& in, pyramid_data_model_agra_OrbitalSurveillanceTaskType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceTaskType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceTaskType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceTargetType& in, pyramid_data_model_agra_OrbitalSurveillanceTargetType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceTargetType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceTargetType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceObjectsType& in, pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceObjectsType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceObjectsType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceObjectType& in, pyramid_data_model_agra_OrbitalSurveillanceObjectType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceObjectType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceObjectType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceObjectBaseType& in, pyramid_data_model_agra_OrbitalSurveillanceObjectBaseType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceObjectBaseType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceObjectBaseType& out);

void to_c(const pyramid::domain_model::agra::SatelliteIdentityChoiceType& in, pyramid_data_model_agra_SatelliteIdentityChoiceType_c* out);
void from_c(const pyramid_data_model_agra_SatelliteIdentityChoiceType_c* in, pyramid::domain_model::agra::SatelliteIdentityChoiceType& out);

void to_c(const pyramid::domain_model::agra::SatelliteIdentityType& in, pyramid_data_model_agra_SatelliteIdentityType_c* out);
void from_c(const pyramid_data_model_agra_SatelliteIdentityType_c* in, pyramid::domain_model::agra::SatelliteIdentityType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceLocationTargetType& in, pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceLocationTargetType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceLocationTargetType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceZoneTargetType& in, pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceZoneTargetType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceZoneTargetType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorTaskType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorTaskType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorTaskType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorTaskBaseType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorTaskBaseType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorTaskBaseType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorTaskBaseType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceCollectionRequirementsType& in, pyramid_data_model_agra_OrbitalSurveillanceCollectionRequirementsType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceCollectionRequirementsType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceCollectionRequirementsType& out);

void to_c(const pyramid::domain_model::agra::MetricCollectionType& in, pyramid_data_model_agra_MetricCollectionType_c* out);
void from_c(const pyramid_data_model_agra_MetricCollectionType_c* in, pyramid::domain_model::agra::MetricCollectionType& out);

void to_c(const pyramid::domain_model::agra::ObservationsPerTrackLimitsType& in, pyramid_data_model_agra_ObservationsPerTrackLimitsType_c* out);
void from_c(const pyramid_data_model_agra_ObservationsPerTrackLimitsType_c* in, pyramid::domain_model::agra::ObservationsPerTrackLimitsType& out);

void to_c(const pyramid::domain_model::agra::SensorCharacterizationChoiceType& in, pyramid_data_model_agra_SensorCharacterizationChoiceType_c* out);
void from_c(const pyramid_data_model_agra_SensorCharacterizationChoiceType_c* in, pyramid::domain_model::agra::SensorCharacterizationChoiceType& out);

void to_c(const pyramid::domain_model::agra::StructureAssessmentCharacterizationType& in, pyramid_data_model_agra_StructureAssessmentCharacterizationType_c* out);
void from_c(const pyramid_data_model_agra_StructureAssessmentCharacterizationType_c* in, pyramid::domain_model::agra::StructureAssessmentCharacterizationType& out);

void to_c(const pyramid::domain_model::agra::SizeEstimationCharacterizationType& in, pyramid_data_model_agra_SizeEstimationCharacterizationType_c* out);
void from_c(const pyramid_data_model_agra_SizeEstimationCharacterizationType_c* in, pyramid::domain_model::agra::SizeEstimationCharacterizationType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorMinimumCollectionRequirementsType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumCollectionRequirementsType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorMinimumCollectionRequirementsType& out);

void to_c(const pyramid::domain_model::agra::IdentificationVerificationCharacterizationType& in, pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c* out);
void from_c(const pyramid_data_model_agra_IdentificationVerificationCharacterizationType_c* in, pyramid::domain_model::agra::IdentificationVerificationCharacterizationType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorReportingCategoriesType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorReportingCategoriesType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorReportingCategoriesType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorReportingCategoriesType& out);

void to_c(const pyramid::domain_model::agra::ReportToType& in, pyramid_data_model_agra_ReportToType_c* out);
void from_c(const pyramid_data_model_agra_ReportToType_c* in, pyramid::domain_model::agra::ReportToType& out);

void to_c(const pyramid::domain_model::agra::AppliesToType& in, pyramid_data_model_agra_AppliesToType_c* out);
void from_c(const pyramid_data_model_agra_AppliesToType_c* in, pyramid::domain_model::agra::AppliesToType& out);

void to_c(const pyramid::domain_model::agra::ContactDetailsType& in, pyramid_data_model_agra_ContactDetailsType_c* out);
void from_c(const pyramid_data_model_agra_ContactDetailsType_c* in, pyramid::domain_model::agra::ContactDetailsType& out);

void to_c(const pyramid::domain_model::agra::OperatorNameType& in, pyramid_data_model_agra_OperatorNameType_c* out);
void from_c(const pyramid_data_model_agra_OperatorNameType_c* in, pyramid::domain_model::agra::OperatorNameType& out);

void to_c(const pyramid::domain_model::agra::UnitIdentityType& in, pyramid_data_model_agra_UnitIdentityType_c* out);
void from_c(const pyramid_data_model_agra_UnitIdentityType_c* in, pyramid::domain_model::agra::UnitIdentityType& out);

void to_c(const pyramid::domain_model::agra::UnitID_Type& in, pyramid_data_model_agra_UnitID_Type_c* out);
void from_c(const pyramid_data_model_agra_UnitID_Type_c* in, pyramid::domain_model::agra::UnitID_Type& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorSensitivityConstraintType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorSensitivityConstraintType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorSensitivityConstraintType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorMinimumSizeType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorMinimumSizeType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorMinimumSizeType& out);

void to_c(const pyramid::domain_model::agra::PercentileRCSType& in, pyramid_data_model_agra_PercentileRCSType_c* out);
void from_c(const pyramid_data_model_agra_PercentileRCSType_c* in, pyramid::domain_model::agra::PercentileRCSType& out);

void to_c(const pyramid::domain_model::agra::OrbitalSurveillanceSensorTargetType& in, pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c* out);
void from_c(const pyramid_data_model_agra_OrbitalSurveillanceSensorTargetType_c* in, pyramid::domain_model::agra::OrbitalSurveillanceSensorTargetType& out);

void to_c(const pyramid::domain_model::agra::SensorPointListType& in, pyramid_data_model_agra_SensorPointListType_c* out);
void from_c(const pyramid_data_model_agra_SensorPointListType_c* in, pyramid::domain_model::agra::SensorPointListType& out);

void to_c(const pyramid::domain_model::agra::AzimuthElevationRangePointType& in, pyramid_data_model_agra_AzimuthElevationRangePointType_c* out);
void from_c(const pyramid_data_model_agra_AzimuthElevationRangePointType_c* in, pyramid::domain_model::agra::AzimuthElevationRangePointType& out);

void to_c(const pyramid::domain_model::agra::SensorPointListType_AzimuthElevationRangePointList_List& in, pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c* out);
void from_c(const pyramid_data_model_agra_SensorPointListType_AzimuthElevationRangePointList_List_c* in, pyramid::domain_model::agra::SensorPointListType_AzimuthElevationRangePointList_List& out);

void to_c(const pyramid::domain_model::agra::RightAscensionDeclinationPointType& in, pyramid_data_model_agra_RightAscensionDeclinationPointType_c* out);
void from_c(const pyramid_data_model_agra_RightAscensionDeclinationPointType_c* in, pyramid::domain_model::agra::RightAscensionDeclinationPointType& out);

void to_c(const pyramid::domain_model::agra::SensorPointListType_RightAscensionDeclinationPointList_List& in, pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c* out);
void from_c(const pyramid_data_model_agra_SensorPointListType_RightAscensionDeclinationPointList_List_c* in, pyramid::domain_model::agra::SensorPointListType_RightAscensionDeclinationPointList_List& out);

void to_c(const pyramid::domain_model::agra::SensorPointListType_Point3DList_List& in, pyramid_data_model_agra_SensorPointListType_Point3DList_List_c* out);
void from_c(const pyramid_data_model_agra_SensorPointListType_Point3DList_List_c* in, pyramid::domain_model::agra::SensorPointListType_Point3DList_List& out);

void to_c(const pyramid::domain_model::agra::ElementSetCloudType& in, pyramid_data_model_agra_ElementSetCloudType_c* out);
void from_c(const pyramid_data_model_agra_ElementSetCloudType_c* in, pyramid::domain_model::agra::ElementSetCloudType& out);

void to_c(const pyramid::domain_model::agra::TLE_Type& in, pyramid_data_model_agra_TLE_Type_c* out);
void from_c(const pyramid_data_model_agra_TLE_Type_c* in, pyramid::domain_model::agra::TLE_Type& out);

void to_c(const pyramid::domain_model::agra::SourceCoverageType& in, pyramid_data_model_agra_SourceCoverageType_c* out);
void from_c(const pyramid_data_model_agra_SourceCoverageType_c* in, pyramid::domain_model::agra::SourceCoverageType& out);

void to_c(const pyramid::domain_model::agra::AngleRateRangeType& in, pyramid_data_model_agra_AngleRateRangeType_c* out);
void from_c(const pyramid_data_model_agra_AngleRateRangeType_c* in, pyramid::domain_model::agra::AngleRateRangeType& out);

void to_c(const pyramid::domain_model::agra::DoubleMinMaxType& in, pyramid_data_model_agra_DoubleMinMaxType_c* out);
void from_c(const pyramid_data_model_agra_DoubleMinMaxType_c* in, pyramid::domain_model::agra::DoubleMinMaxType& out);

void to_c(const pyramid::domain_model::agra::CapabilityCoverageAreaID_Type& in, pyramid_data_model_agra_CapabilityCoverageAreaID_Type_c* out);
void from_c(const pyramid_data_model_agra_CapabilityCoverageAreaID_Type_c* in, pyramid::domain_model::agra::CapabilityCoverageAreaID_Type& out);

void to_c(const pyramid::domain_model::agra::PO_TaskType& in, pyramid_data_model_agra_PO_TaskType_c* out);
void from_c(const pyramid_data_model_agra_PO_TaskType_c* in, pyramid::domain_model::agra::PO_TaskType& out);

void to_c(const pyramid::domain_model::agra::PointingType& in, pyramid_data_model_agra_PointingType_c* out);
void from_c(const pyramid_data_model_agra_PointingType_c* in, pyramid::domain_model::agra::PointingType& out);

void to_c(const pyramid::domain_model::agra::PointingType_Geospatial_List& in, pyramid_data_model_agra_PointingType_Geospatial_List_c* out);
void from_c(const pyramid_data_model_agra_PointingType_Geospatial_List_c* in, pyramid::domain_model::agra::PointingType_Geospatial_List& out);

void to_c(const pyramid::domain_model::agra::LOS_D_Type& in, pyramid_data_model_agra_LOS_D_Type_c* out);
void from_c(const pyramid_data_model_agra_LOS_D_Type_c* in, pyramid::domain_model::agra::LOS_D_Type& out);

void to_c(const pyramid::domain_model::agra::LOS_VariableB_Type& in, pyramid_data_model_agra_LOS_VariableB_Type_c* out);
void from_c(const pyramid_data_model_agra_LOS_VariableB_Type_c* in, pyramid::domain_model::agra::LOS_VariableB_Type& out);

void to_c(const pyramid::domain_model::agra::LOS_VariableA_Type& in, pyramid_data_model_agra_LOS_VariableA_Type_c* out);
void from_c(const pyramid_data_model_agra_LOS_VariableA_Type_c* in, pyramid::domain_model::agra::LOS_VariableA_Type& out);

void to_c(const pyramid::domain_model::agra::LOS_RatesType& in, pyramid_data_model_agra_LOS_RatesType_c* out);
void from_c(const pyramid_data_model_agra_LOS_RatesType_c* in, pyramid::domain_model::agra::LOS_RatesType& out);

void to_c(const pyramid::domain_model::agra::PO_AirTargetVolumeCommandType& in, pyramid_data_model_agra_PO_AirTargetVolumeCommandType_c* out);
void from_c(const pyramid_data_model_agra_PO_AirTargetVolumeCommandType_c* in, pyramid::domain_model::agra::PO_AirTargetVolumeCommandType& out);

void to_c(const pyramid::domain_model::agra::PO_AirTargetVolumeType& in, pyramid_data_model_agra_PO_AirTargetVolumeType_c* out);
void from_c(const pyramid_data_model_agra_PO_AirTargetVolumeType_c* in, pyramid::domain_model::agra::PO_AirTargetVolumeType& out);

void to_c(const pyramid::domain_model::agra::PO_AirVolumeSensorReferencedType& in, pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c* out);
void from_c(const pyramid_data_model_agra_PO_AirVolumeSensorReferencedType_c* in, pyramid::domain_model::agra::PO_AirVolumeSensorReferencedType& out);

void to_c(const pyramid::domain_model::agra::PointingType_Volume_List& in, pyramid_data_model_agra_PointingType_Volume_List_c* out);
void from_c(const pyramid_data_model_agra_PointingType_Volume_List_c* in, pyramid::domain_model::agra::PointingType_Volume_List& out);

void to_c(const pyramid::domain_model::agra::RefuelTaskType& in, pyramid_data_model_agra_RefuelTaskType_c* out);
void from_c(const pyramid_data_model_agra_RefuelTaskType_c* in, pyramid::domain_model::agra::RefuelTaskType& out);

void to_c(const pyramid::domain_model::agra::IdentityKindAssetType& in, pyramid_data_model_agra_IdentityKindAssetType_c* out);
void from_c(const pyramid_data_model_agra_IdentityKindAssetType_c* in, pyramid::domain_model::agra::IdentityKindAssetType& out);

void to_c(const pyramid::domain_model::agra::RequirementPlanningCandidateType& in, pyramid_data_model_agra_RequirementPlanningCandidateType_c* out);
void from_c(const pyramid_data_model_agra_RequirementPlanningCandidateType_c* in, pyramid::domain_model::agra::RequirementPlanningCandidateType& out);

void to_c(const pyramid::domain_model::agra::PlanningCandidateBaseType& in, pyramid_data_model_agra_PlanningCandidateBaseType_c* out);
void from_c(const pyramid_data_model_agra_PlanningCandidateBaseType_c* in, pyramid::domain_model::agra::PlanningCandidateBaseType& out);

void to_c(const pyramid::domain_model::agra::ConstrainingPlansType& in, pyramid_data_model_agra_ConstrainingPlansType_c* out);
void from_c(const pyramid_data_model_agra_ConstrainingPlansType_c* in, pyramid::domain_model::agra::ConstrainingPlansType& out);

void to_c(const pyramid::domain_model::agra::TaskPlanConstraintType& in, pyramid_data_model_agra_TaskPlanConstraintType_c* out);
void from_c(const pyramid_data_model_agra_TaskPlanConstraintType_c* in, pyramid::domain_model::agra::TaskPlanConstraintType& out);

void to_c(const pyramid::domain_model::agra::OtherSystemConstrainingPlansType& in, pyramid_data_model_agra_OtherSystemConstrainingPlansType_c* out);
void from_c(const pyramid_data_model_agra_OtherSystemConstrainingPlansType_c* in, pyramid::domain_model::agra::OtherSystemConstrainingPlansType& out);

void to_c(const pyramid::domain_model::agra::SAR_TaskType& in, pyramid_data_model_agra_SAR_TaskType_c* out);
void from_c(const pyramid_data_model_agra_SAR_TaskType_c* in, pyramid::domain_model::agra::SAR_TaskType& out);

void to_c(const pyramid::domain_model::agra::SAR_TaskTargetType& in, pyramid_data_model_agra_SAR_TaskTargetType_c* out);
void from_c(const pyramid_data_model_agra_SAR_TaskTargetType_c* in, pyramid::domain_model::agra::SAR_TaskTargetType& out);

void to_c(const pyramid::domain_model::agra::SAR_TargetType& in, pyramid_data_model_agra_SAR_TargetType_c* out);
void from_c(const pyramid_data_model_agra_SAR_TargetType_c* in, pyramid::domain_model::agra::SAR_TargetType& out);

void to_c(const pyramid::domain_model::agra::ISAR_TargetType& in, pyramid_data_model_agra_ISAR_TargetType_c* out);
void from_c(const pyramid_data_model_agra_ISAR_TargetType_c* in, pyramid::domain_model::agra::ISAR_TargetType& out);

void to_c(const pyramid::domain_model::agra::SMTI_TaskType& in, pyramid_data_model_agra_SMTI_TaskType_c* out);
void from_c(const pyramid_data_model_agra_SMTI_TaskType_c* in, pyramid::domain_model::agra::SMTI_TaskType& out);

void to_c(const pyramid::domain_model::agra::StrikeTaskType& in, pyramid_data_model_agra_StrikeTaskType_c* out);
void from_c(const pyramid_data_model_agra_StrikeTaskType_c* in, pyramid::domain_model::agra::StrikeTaskType& out);

void to_c(const pyramid::domain_model::agra::TargetInformationType& in, pyramid_data_model_agra_TargetInformationType_c* out);
void from_c(const pyramid_data_model_agra_TargetInformationType_c* in, pyramid::domain_model::agra::TargetInformationType& out);

void to_c(const pyramid::domain_model::agra::StrikeTaskReleaseConstraintsType& in, pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_StrikeTaskReleaseConstraintsType_c* in, pyramid::domain_model::agra::StrikeTaskReleaseConstraintsType& out);

void to_c(const pyramid::domain_model::agra::AreaConstraintsType& in, pyramid_data_model_agra_AreaConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_AreaConstraintsType_c* in, pyramid::domain_model::agra::AreaConstraintsType& out);

void to_c(const pyramid::domain_model::agra::AltitudeConstraintsType& in, pyramid_data_model_agra_AltitudeConstraintsType_c* out);
void from_c(const pyramid_data_model_agra_AltitudeConstraintsType_c* in, pyramid::domain_model::agra::AltitudeConstraintsType& out);

void to_c(const pyramid::domain_model::agra::SystemDeploymentTaskType& in, pyramid_data_model_agra_SystemDeploymentTaskType_c* out);
void from_c(const pyramid_data_model_agra_SystemDeploymentTaskType_c* in, pyramid::domain_model::agra::SystemDeploymentTaskType& out);

void to_c(const pyramid::domain_model::agra::DeployableSystemIdentityType& in, pyramid_data_model_agra_DeployableSystemIdentityType_c* out);
void from_c(const pyramid_data_model_agra_DeployableSystemIdentityType_c* in, pyramid::domain_model::agra::DeployableSystemIdentityType& out);

void to_c(const pyramid::domain_model::agra::TacticalOrderTaskType& in, pyramid_data_model_agra_TacticalOrderTaskType_c* out);
void from_c(const pyramid_data_model_agra_TacticalOrderTaskType_c* in, pyramid::domain_model::agra::TacticalOrderTaskType& out);

void to_c(const pyramid::domain_model::agra::WeatherRadarTaskType& in, pyramid_data_model_agra_WeatherRadarTaskType_c* out);
void from_c(const pyramid_data_model_agra_WeatherRadarTaskType_c* in, pyramid::domain_model::agra::WeatherRadarTaskType& out);

void to_c(const pyramid::domain_model::agra::MA_ConstraintID_Type& in, pyramid_data_model_agra_MA_ConstraintID_Type_c* out);
void from_c(const pyramid_data_model_agra_MA_ConstraintID_Type_c* in, pyramid::domain_model::agra::MA_ConstraintID_Type& out);

void to_c(const pyramid::domain_model::agra::MA_TaskStatusMT& in, pyramid_data_model_agra_MA_TaskStatusMT_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskStatusMT_c* in, pyramid::domain_model::agra::MA_TaskStatusMT& out);

void to_c(const pyramid::domain_model::agra::MA_TaskStatusMDT& in, pyramid_data_model_agra_MA_TaskStatusMDT_c* out);
void from_c(const pyramid_data_model_agra_MA_TaskStatusMDT_c* in, pyramid::domain_model::agra::MA_TaskStatusMDT& out);

void to_c(const pyramid::domain_model::agra::MissionContingencyAlertMT& in, pyramid_data_model_agra_MissionContingencyAlertMT_c* out);
void from_c(const pyramid_data_model_agra_MissionContingencyAlertMT_c* in, pyramid::domain_model::agra::MissionContingencyAlertMT& out);

void to_c(const pyramid::domain_model::agra::MissionContingencyAlertMDT& in, pyramid_data_model_agra_MissionContingencyAlertMDT_c* out);
void from_c(const pyramid_data_model_agra_MissionContingencyAlertMDT_c* in, pyramid::domain_model::agra::MissionContingencyAlertMDT& out);

void to_c(const pyramid::domain_model::agra::MissionContingencyConditionType& in, pyramid_data_model_agra_MissionContingencyConditionType_c* out);
void from_c(const pyramid_data_model_agra_MissionContingencyConditionType_c* in, pyramid::domain_model::agra::MissionContingencyConditionType& out);

void to_c(const pyramid::domain_model::agra::ConflictType& in, pyramid_data_model_agra_ConflictType_c* out);
void from_c(const pyramid_data_model_agra_ConflictType_c* in, pyramid::domain_model::agra::ConflictType& out);

void to_c(const pyramid::domain_model::agra::ConflictLocationType& in, pyramid_data_model_agra_ConflictLocationType_c* out);
void from_c(const pyramid_data_model_agra_ConflictLocationType_c* in, pyramid::domain_model::agra::ConflictLocationType& out);

void to_c(const pyramid::domain_model::agra::RoutePlanSegmentReferenceType& in, pyramid_data_model_agra_RoutePlanSegmentReferenceType_c* out);
void from_c(const pyramid_data_model_agra_RoutePlanSegmentReferenceType_c* in, pyramid::domain_model::agra::RoutePlanSegmentReferenceType& out);

void to_c(const pyramid::domain_model::agra::RequirementsReferenceType& in, pyramid_data_model_agra_RequirementsReferenceType_c* out);
void from_c(const pyramid_data_model_agra_RequirementsReferenceType_c* in, pyramid::domain_model::agra::RequirementsReferenceType& out);

void to_c(const pyramid::domain_model::agra::RequirementsReferenceLockableType& in, pyramid_data_model_agra_RequirementsReferenceLockableType_c* out);
void from_c(const pyramid_data_model_agra_RequirementsReferenceLockableType_c* in, pyramid::domain_model::agra::RequirementsReferenceLockableType& out);

void to_c(const pyramid::domain_model::agra::AutonomousActionStatusChoiceType& in, pyramid_data_model_agra_AutonomousActionStatusChoiceType_c* out);
void from_c(const pyramid_data_model_agra_AutonomousActionStatusChoiceType_c* in, pyramid::domain_model::agra::AutonomousActionStatusChoiceType& out);

void to_c(const pyramid::domain_model::agra::AutonomousPlanningActionStatusType& in, pyramid_data_model_agra_AutonomousPlanningActionStatusType_c* out);
void from_c(const pyramid_data_model_agra_AutonomousPlanningActionStatusType_c* in, pyramid::domain_model::agra::AutonomousPlanningActionStatusType& out);

void to_c(const pyramid::domain_model::agra::AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List& in, pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c* out);
void from_c(const pyramid_data_model_agra_AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List_c* in, pyramid::domain_model::agra::AutonomousActionStatusChoiceType_AutonomousPlanningActionStatus_List& out);

} // namespace pyramid::cabi
