#include "agra_p3_example/c2_component.hpp"
#include "agra_p3_example/process_runtime.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  try {
    agra_p3_example::ProcessRuntime runtime(
        argc, argv, {AGRA_P3_C2_JSON_CODEC_PLUGIN_PATH},
        {{"action_command_request",
          agra_p3_example::c2::MaActioncommandRequestPortClient::
              deploymentDescriptor()},
         {"action_information",
          agra_p3_example::c2::MaActionInformationPortSink::
              deploymentDescriptor()}});
    agra_p3_example::C2Component component(runtime.executor());
    return runtime.run(component);
  } catch (const std::exception& error) {
    std::cerr << "c2: " << error.what() << std::endl;
    return 1;
  }
}
