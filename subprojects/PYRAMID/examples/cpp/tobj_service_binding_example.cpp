// Entry point for the C++ PYRAMID service-binding example.
//
// The interesting pieces are split into the neighbouring files so users can
// copy the part they need: handler, provider component, client component, and
// shared-memory runner.

#include "tobj_shared_memory_example.hpp"

#include "pyramid_services_tactical_objects_provided.hpp"

#include <iostream>

namespace svc = pyramid::components::tactical_objects::services::provided;

int main() {
  // Run the example in FlatBuffers mode so the test proves the generated
  // service binding is doing real typed encode/decode work.
  if (!tobj_example::runSharedMemoryExample(svc::kFlatBuffersContentType)) {
    std::cerr << "shared-memory service-binding example failed\n";
    return 1;
  }

  std::cout << "service-binding example completed\n";
  return 0;
}
