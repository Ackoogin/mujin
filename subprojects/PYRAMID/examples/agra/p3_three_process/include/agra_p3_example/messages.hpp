#pragma once

#include "pyramid_services_agra_c2_provided_components.hpp"
#include "pyramid_services_agra_ms_consumed_components.hpp"
#include "pyramid_services_agra_ms_provided_components.hpp"

#include <string>

namespace agra_p3_example {

namespace c2 = pyramid::components::agra::c2::services::provided;
namespace ms_request = pyramid::components::agra::ms::services::consumed;
namespace ms_information = pyramid::components::agra::ms::services::provided;

/// \brief Build an MA-Action publication from Mission Autonomy to C2.
c2::MA_Action_Service_Information makeActionInformation(unsigned sequence);

/// \brief Return the action identifier carried by an information publication.
std::string actionId(const c2::MA_Action_Service_Information& information);

/// \brief Build the C2 request commanding Mission Autonomy to create an action.
c2::MA_ActionCommand_Service_Request makeActionRequest();

/// \brief Return the command identifier carried by a Create request.
std::string actionCommandId(
    const c2::MA_ActionCommand_Service_Request& request);

/// \brief Build a task publication from Mission Autonomy to a Mission System.
ms_information::MA_Task_Service_Information makeTaskInformation(
    unsigned sequence);

/// \brief Return the task identifier carried by an information publication.
std::string taskId(
    const ms_information::MA_Task_Service_Information& information);

/// \brief Build Mission Autonomy's task Create request to a Mission System.
ms_request::MA_TaskCommand_Service_Request makeTaskRequest(
    const std::string& action_command_id);

/// \brief Return the command identifier carried by a task Create request.
std::string taskCommandId(
    const ms_request::MA_TaskCommand_Service_Request& request);

}  // namespace agra_p3_example
