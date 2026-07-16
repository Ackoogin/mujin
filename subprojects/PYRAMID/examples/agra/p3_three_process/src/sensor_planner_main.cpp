#include "agra_p3_example/process_runtime.hpp"
#include "agra_p3_example/sensor_planner_component.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  try {
    agra_p3_example::ProcessRuntime runtime(
        argc, argv, AGRA_P3_C2_JSON_CODEC_PLUGIN_PATH);
    agra_p3_example::SensorPlannerComponent component(runtime.executor());
    return runtime.run(component);
  } catch (const std::exception& error) {
    std::cerr << "sensor_planner: " << error.what() << std::endl;
    return 1;
  }
}
