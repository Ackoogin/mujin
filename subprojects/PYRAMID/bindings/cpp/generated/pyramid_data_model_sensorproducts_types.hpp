// Auto-generated types header
// Generated from: pyramid.data_model.sensorproducts.proto by generate_bindings.py (types)
// Namespace: pyramid::domain_model::sensorproducts
#pragma once

#include <cstdint>
#include <tl/optional.hpp>
#include <string>
#include <vector>
#include "pyramid_data_model_common_types.hpp"

namespace pyramid::domain_model::sensorproducts {


struct RadarDisplayProductRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
};

struct RadarProductRequirement {
    pyramid::domain_model::common::Entity base = {};  // from Requirement
    pyramid::domain_model::common::Achievement status = {};  // from Requirement
};

} // namespace pyramid::domain_model::sensorproducts
