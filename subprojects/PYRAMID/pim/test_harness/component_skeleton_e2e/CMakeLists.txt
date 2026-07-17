cmake_minimum_required(VERSION 3.21)
project(component_skeleton_e2e LANGUAGES CXX)

set(PYRAMID_SDK "" CACHE PATH "Path to the PYRAMID SDK")
if(NOT PYRAMID_SDK)
  message(FATAL_ERROR "Set PYRAMID_SDK to the SDK directory.")
endif()

set(PYRAMID_SDK_SERVICE_BINDINGS
    "pim_osprey_sensors_provided" CACHE STRING "" FORCE)
set(PYRAMID_SDK_COMPONENTS
    "pim_osprey.sensors" CACHE STRING "" FORCE)
set(PYRAMID_SDK_ENABLE_FLATBUFFERS OFF CACHE BOOL "" FORCE)
add_subdirectory(${PYRAMID_SDK}/sdk_project
                 ${CMAKE_CURRENT_BINARY_DIR}/pyramid_sdk)

add_executable(component_skeleton_e2e component_skeleton_e2e.cpp)
target_link_libraries(component_skeleton_e2e PRIVATE
    pyramid_component_skeletons
    pyramid_generated_services
    pcl_core)
target_compile_definitions(component_skeleton_e2e PRIVATE
    "PYRAMID_COMPONENT_CODEC_PLUGIN_PATH=\"$<TARGET_FILE:pyramid_codec_json_pim_osprey_sensors>\"")
add_dependencies(component_skeleton_e2e
    pyramid_codec_json_pim_osprey_sensors)
if(MSVC)
  target_compile_options(component_skeleton_e2e PRIVATE /bigobj)
endif()
