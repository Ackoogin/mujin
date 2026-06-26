#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_sensors_cabi.h"
#include "pyramid_data_model_base_cabi_marshal.hpp"
#include "pyramid_data_model_common_cabi_marshal.hpp"
#include "pyramid_data_model_radar_cabi_marshal.hpp"
#include "pyramid_data_model_sensorproducts_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::InterpretationRequirement& in, pyramid_InterpretationRequirement_c* out);
void from_c(const pyramid_InterpretationRequirement_c* in, pyramid::domain_model::InterpretationRequirement& out);

void to_c(const pyramid::domain_model::ManualTrackRequirement& in, pyramid_ManualTrackRequirement_c* out);
void from_c(const pyramid_ManualTrackRequirement_c* in, pyramid::domain_model::ManualTrackRequirement& out);

void to_c(const pyramid::domain_model::ATIRequirement& in, pyramid_ATIRequirement_c* out);
void from_c(const pyramid_ATIRequirement_c* in, pyramid::domain_model::ATIRequirement& out);

void to_c(const pyramid::domain_model::TrackProvisionRequirement& in, pyramid_TrackProvisionRequirement_c* out);
void from_c(const pyramid_TrackProvisionRequirement_c* in, pyramid::domain_model::TrackProvisionRequirement& out);

void to_c(const pyramid::domain_model::ObjectEvidenceProvisionRequirement& in, pyramid_ObjectEvidenceProvisionRequirement_c* out);
void from_c(const pyramid_ObjectEvidenceProvisionRequirement_c* in, pyramid::domain_model::ObjectEvidenceProvisionRequirement& out);

void to_c(const pyramid::domain_model::ObjectAquisitionRequirement& in, pyramid_ObjectAquisitionRequirement_c* out);
void from_c(const pyramid_ObjectAquisitionRequirement_c* in, pyramid::domain_model::ObjectAquisitionRequirement& out);

void to_c(const pyramid::domain_model::SensorObject& in, pyramid_SensorObject_c* out);
void from_c(const pyramid_SensorObject_c* in, pyramid::domain_model::SensorObject& out);

void to_c(const pyramid::domain_model::RadarModeChangeRequirement& in, pyramid_RadarModeChangeRequirement_c* out);
void from_c(const pyramid_RadarModeChangeRequirement_c* in, pyramid::domain_model::RadarModeChangeRequirement& out);

} // namespace pyramid::cabi
