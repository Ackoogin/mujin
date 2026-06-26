#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_data_model_common_cabi.h"
#include "pyramid_data_model_base_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::domain_model::GeodeticPosition& in, pyramid_GeodeticPosition_c* out);
void from_c(const pyramid_GeodeticPosition_c* in, pyramid::domain_model::GeodeticPosition& out);

void to_c(const pyramid::domain_model::PolyArea& in, pyramid_PolyArea_c* out);
void from_c(const pyramid_PolyArea_c* in, pyramid::domain_model::PolyArea& out);

void to_c(const pyramid::domain_model::Achievement& in, pyramid_Achievement_c* out);
void from_c(const pyramid_Achievement_c* in, pyramid::domain_model::Achievement& out);

void to_c(const pyramid::domain_model::Requirement& in, pyramid_Requirement_c* out);
void from_c(const pyramid_Requirement_c* in, pyramid::domain_model::Requirement& out);

void to_c(const pyramid::domain_model::Capability& in, pyramid_Capability_c* out);
void from_c(const pyramid_Capability_c* in, pyramid::domain_model::Capability& out);

void to_c(const pyramid::domain_model::Entity& in, pyramid_Entity_c* out);
void from_c(const pyramid_Entity_c* in, pyramid::domain_model::Entity& out);

void to_c(const pyramid::domain_model::CircleArea& in, pyramid_CircleArea_c* out);
void from_c(const pyramid_CircleArea_c* in, pyramid::domain_model::CircleArea& out);

void to_c(const pyramid::domain_model::Point& in, pyramid_Point_c* out);
void from_c(const pyramid_Point_c* in, pyramid::domain_model::Point& out);

void to_c(const pyramid::domain_model::Contraint& in, pyramid_Contraint_c* out);
void from_c(const pyramid_Contraint_c* in, pyramid::domain_model::Contraint& out);

void to_c(const pyramid::domain_model::Ack& in, pyramid_Ack_c* out);
void from_c(const pyramid_Ack_c* in, pyramid::domain_model::Ack& out);

void to_c(const pyramid::domain_model::Query& in, pyramid_Query_c* out);
void from_c(const pyramid_Query_c* in, pyramid::domain_model::Query& out);

} // namespace pyramid::cabi
