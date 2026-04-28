#pragma once

#include <cstddef>

namespace pyramid::components::tactical_objects::services::provided {

class ServiceHandler;

enum class ServiceChannel {
    ReadMatch,
    CreateRequirement,
    ReadRequirement,
    UpdateRequirement,
    DeleteRequirement,
    ReadDetail,
};

void dispatch(ServiceHandler& handler,
              ServiceChannel channel,
              const void* request_buf,
              size_t request_size,
              const char* content_type,
              void** response_buf,
              size_t* response_size);

}  // namespace pyramid::components::tactical_objects::services::provided

namespace pyramid::components::tactical_objects::services::consumed {

class ServiceHandler;

enum class ServiceChannel {
    ReadDetail,
    CreateRequirement,
    ReadRequirement,
    UpdateRequirement,
    DeleteRequirement,
    ReadCapability,
};

void dispatch(ServiceHandler& handler,
              ServiceChannel channel,
              const void* request_buf,
              size_t request_size,
              const char* content_type,
              void** response_buf,
              size_t* response_size);

}  // namespace pyramid::components::tactical_objects::services::consumed
