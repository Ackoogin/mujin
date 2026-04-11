#pragma once

#include <cstddef>

namespace pyramid::services::tactical_objects::provided {

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

}  // namespace pyramid::services::tactical_objects::provided

namespace pyramid::services::tactical_objects::consumed {

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

}  // namespace pyramid::services::tactical_objects::consumed
