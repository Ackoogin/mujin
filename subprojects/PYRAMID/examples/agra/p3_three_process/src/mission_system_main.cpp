#include "agra_p3_example/mission_system_component.hpp"

#include <pcl/process_runtime.hpp>

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  try {
    pcl::ProcessRuntime runtime(
        argc, argv, {AGRA_P3_MS_JSON_CODEC_PLUGIN_PATH},
        {{"task_command_request",
          agra_p3_example::ms_request::MaTaskcommandRequestPortProvider::
              deploymentDescriptor()},
         {"task_information",
          agra_p3_example::ms_information::MaTaskInformationPortSink::
              deploymentDescriptor()}});
    agra_p3_example::MissionSystemComponent component(runtime.executor());
    return runtime.run(component);
  } catch (const std::exception& error) {
    std::cerr << "mission_system: " << error.what() << std::endl;
    return 1;
  }
}
