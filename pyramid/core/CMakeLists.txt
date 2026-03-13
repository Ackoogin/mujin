cmake_minimum_required(VERSION 3.10)

add_library(pyramid_core)
add_library(pyramid::core ALIAS pyramid_core)

# Specify source files
target_sources(pyramid_core
    PRIVATE
        src/Core.cpp
        src/logging/Logger.cpp
        src/uuid/UUIDHelper.cpp
        src/event/EventBus.cpp
        src/job/JobSystem.cpp
        # Add other .cpp files here
    PUBLIC
        # Public headers if any beyond the include directory
)

# Specify include directories
target_include_directories(pyramid_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${CMAKE_CURRENT_SOURCE_DIR}/external # Add external directory for sha1 etc.
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # Private include directories if needed
)

# Specify dependencies (if any)
# target_link_libraries(pyramid_core
#     PUBLIC
#         # Public dependencies
#     PRIVATE
#         # Private dependencies
# )

target_compile_features(pyramid_core PUBLIC cxx_std_14)