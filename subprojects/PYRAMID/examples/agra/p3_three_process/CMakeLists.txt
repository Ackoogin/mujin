cmake_minimum_required(VERSION 3.21)
project(agra_p3_three_process_example LANGUAGES CXX)

set(PYRAMID_P3_SDK "" CACHE PATH
    "Path to the pcl_pyramid_sdk_agra_p3 distribution")
if(NOT PYRAMID_P3_SDK)
  message(FATAL_ERROR
      "Set PYRAMID_P3_SDK to the pcl_pyramid_sdk_agra_p3 directory.")
endif()
get_filename_component(PYRAMID_P3_SDK "${PYRAMID_P3_SDK}" ABSOLUTE)
if(NOT EXISTS "${PYRAMID_P3_SDK}/sdk_project/CMakeLists.txt")
  message(FATAL_ERROR
      "${PYRAMID_P3_SDK} is not a PCL/PYRAMID SDK distribution.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The formal P3 seam uses the JSON backend. Its unions are intentionally not
# sent through the SDK's current FlatBuffers backend.
set(PYRAMID_SDK_ENABLE_FLATBUFFERS OFF CACHE BOOL "" FORCE)
set(PYRAMID_SDK_SERVICE_BINDINGS
    "agra_c2_consumed;agra_c2_provided;agra_ms_consumed;agra_ms_provided"
    CACHE STRING "" FORCE)
add_subdirectory("${PYRAMID_P3_SDK}/sdk_project"
                 "${CMAKE_CURRENT_BINARY_DIR}/pyramid_sdk" EXCLUDE_FROM_ALL)

if(NOT TARGET pyramid_codec_json_agra_c2 OR
   NOT TARGET pyramid_codec_json_agra_ms)
  message(FATAL_ERROR
      "The selected SDK was not generated from the A-GRA P3 seam. Run the "
      "SDK's scripts/generate_bindings command before configuring this example.")
endif()

add_library(agra_p3_example_support STATIC
    src/messages.cpp)
target_include_directories(agra_p3_example_support PUBLIC include)
target_link_libraries(agra_p3_example_support PUBLIC
    pyramid_generated_services
    pcl_core)
if(MSVC)
  target_compile_options(agra_p3_example_support PRIVATE /bigobj)
endif()

function(add_agra_p3_process target component_source main_source)
  add_executable(${target} ${component_source} ${main_source})
  target_link_libraries(${target} PRIVATE agra_p3_example_support)
  target_compile_definitions(${target} PRIVATE
      "AGRA_P3_C2_JSON_CODEC_PLUGIN_PATH=\"$<TARGET_FILE:pyramid_codec_json_agra_c2>\""
      "AGRA_P3_MS_JSON_CODEC_PLUGIN_PATH=\"$<TARGET_FILE:pyramid_codec_json_agra_ms>\"")
  add_dependencies(${target}
      pyramid_codec_json_agra_c2
      pyramid_codec_json_agra_ms)
  if(MSVC)
    target_compile_options(${target} PRIVATE /bigobj)
    # The complete P3 value types can exceed the Windows 1 MiB default stack
    # while generated facades encode or decode them on application threads.
    target_link_options(${target} PRIVATE "/STACK:33554432")
  endif()
endfunction()

add_agra_p3_process(
    agra_p3_c2
    src/c2_component.cpp
    src/c2_main.cpp)
add_agra_p3_process(
    agra_p3_mission_autonomy
    src/mission_autonomy_component.cpp
    src/mission_autonomy_main.cpp)
add_agra_p3_process(
    agra_p3_mission_system
    src/mission_system_component.cpp
    src/mission_system_main.cpp)
