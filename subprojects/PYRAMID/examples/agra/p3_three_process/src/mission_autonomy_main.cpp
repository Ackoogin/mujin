#include "agra_p3_example/mission_autonomy_component.hpp"

#include <pcl/process_runtime.hpp>

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  try {
    pcl::ProcessRuntime runtime(
        argc, argv,
        {AGRA_P3_C2_JSON_CODEC_PLUGIN_PATH,
         AGRA_P3_MS_JSON_CODEC_PLUGIN_PATH},
        {{"c2_action_command_request",
          agra_p3_example::c2::MaActioncommandRequestPortProvider::
              deploymentDescriptor()},
         {"c2_action_information",
          agra_p3_example::c2::MaActionInformationPortSource::
              deploymentDescriptor()},
         {"ms_task_command_request",
          agra_p3_example::ms_request::MaTaskcommandRequestPortClient::
              deploymentDescriptor()},
         {"ms_task_information",
          agra_p3_example::ms_information::MaTaskInformationPortSource::
              deploymentDescriptor()}});
    agra_p3_example::MissionAutonomyComponent component(runtime.executor());
    return runtime.run(component);
  } catch (const std::exception& error) {
    std::cerr << "mission_autonomy: " << error.what() << std::endl;
    return 1;
  }
}
