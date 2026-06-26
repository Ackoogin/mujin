#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_sensorproducts_cabi.h"
#include "pyramid_data_model_common_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::RadarDisplayProductRequirement& in, pyramid_RadarDisplayProductRequirement_c* out);
void from_c(const pyramid_RadarDisplayProductRequirement_c* in, pyramid::domain_model::RadarDisplayProductRequirement& out);

void to_c(const pyramid::domain_model::RadarProductRequirement& in, pyramid_RadarProductRequirement_c* out);
void from_c(const pyramid_RadarProductRequirement_c* in, pyramid::domain_model::RadarProductRequirement& out);

} // namespace pyramid::cabi
