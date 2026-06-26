#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_tactical_cabi.h"
#include "pyramid_data_model_base_cabi_marshal.hpp"
#include "pyramid_data_model_common_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::ObjectDetail& in, pyramid_ObjectDetail_c* out);
void from_c(const pyramid_ObjectDetail_c* in, pyramid::domain_model::ObjectDetail& out);

void to_c(const pyramid::domain_model::ObjectEvidenceRequirement& in, pyramid_ObjectEvidenceRequirement_c* out);
void from_c(const pyramid_ObjectEvidenceRequirement_c* in, pyramid::domain_model::ObjectEvidenceRequirement& out);

void to_c(const pyramid::domain_model::ObjectInterestRequirement& in, pyramid_ObjectInterestRequirement_c* out);
void from_c(const pyramid_ObjectInterestRequirement_c* in, pyramid::domain_model::ObjectInterestRequirement& out);

void to_c(const pyramid::domain_model::ObjectMatch& in, pyramid_ObjectMatch_c* out);
void from_c(const pyramid_ObjectMatch_c* in, pyramid::domain_model::ObjectMatch& out);

} // namespace pyramid::cabi
