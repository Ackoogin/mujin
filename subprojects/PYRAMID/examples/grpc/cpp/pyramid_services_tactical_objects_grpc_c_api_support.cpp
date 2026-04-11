#include "pyramid_services_tactical_objects_grpc_c_api_support.hpp"

extern "C" {

void pyramid_services_tactical_objects_grpc_free_string(void* value) {
  std::free(value);
}

}  // extern "C"
