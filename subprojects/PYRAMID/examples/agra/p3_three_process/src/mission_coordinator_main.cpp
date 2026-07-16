#include "agra_p3_example/mission_coordinator_component.hpp"
#include "agra_p3_example/process_runtime.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  try {
    agra_p3_example::ProcessRuntime runtime(
        argc, argv, AGRA_P3_C2_JSON_CODEC_PLUGIN_PATH);
    agra_p3_example::MissionCoordinatorComponent component;
    return runtime.run(component);
  } catch (const std::exception& error) {
    std::cerr << "mission_coordinator: " << error.what() << std::endl;
    return 1;
  }
}
