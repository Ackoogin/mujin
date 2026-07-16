#pragma once

#include "pyramid_services_agra_c2_provided.hpp"
#include "pyramid_components_agra_c2_services_provided_codec.hpp"
#include "pyramid_data_model_agra_port_grammar_codec.hpp"

#include <string>

namespace agra_p3_example {

namespace p3 = pyramid::components::agra::c2::services::provided;

/// \brief Build an MA-Action information publication for one component.
p3::MA_Action_Service_Information makeActionInformation(
    const std::string& component_name, unsigned sequence);

/// \brief Return the action identifier carried by an information publication.
std::string actionId(const p3::MA_Action_Service_Information& information);

/// \brief Build a mission-action Create request with a stable command ID.
p3::MA_ActionCommand_Service_Request makeActionRequest(
    const std::string& component_name);

/// \brief Return the command identifier carried by a Create request.
std::string commandId(const p3::MA_ActionCommand_Service_Request& request);

}  // namespace agra_p3_example
