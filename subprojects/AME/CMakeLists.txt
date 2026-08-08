cmake_minimum_required(VERSION 3.21)

# This directory builds two ways.
#
# Inside the unmanned workspace it is pulled in with add_subdirectory(), shares
# the workspace's dependencies, and builds the PCL component layer and the
# PYRAMID bridge alongside everything else.
#
# On its own — copied out for deployment, or opened directly in an IDE — it is a
# complete CMake project that needs neither PCL nor PYRAMID on disk. The core
# engine, the graphical authoring tool, ame_mission_cli and the contingency
# verifier all build in that configuration, because none of them depends on
# either. What is left out is the component layer that genuinely does: see
# AME_BUILD_PCL_COMPONENTS below.
#
# AME_STANDALONE tells the two apart. Nothing else in the build should test how
# it was entered.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(AME_STANDALONE ON)
else()
    set(AME_STANDALONE OFF)
endif()

set(AME_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${AME_ROOT_DIR}/cmake")

# A real saved project for the authoring tool's offscreen self-test to open.
set(AME_SAMPLE_PROJECT
    "${AME_ROOT_DIR}/domains/vehicle_autonomy/vehicle-autonomy.ameproj.json")

if(AME_STANDALONE)
    project(ame LANGUAGES C CXX)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
    endif()

    # The Foxglove bridge is off by default here. It is for watching a live
    # system, which is not what a standalone AME build is usually for, and
    # leaving it off keeps websocketpp and Asio out of the build.
    option(AME_FOXGLOVE "Enable Foxglove WebSocket bridge for live monitoring" OFF)
    option(AME_NEURO  "Neuro-symbolic core infrastructure (ame_neuro)" OFF)
    option(AME_LLM    "LLM backends via libcurl (implies AME_NEURO)"   OFF)
    option(AME_NEURAL "ONNX Runtime backends (implies AME_NEURO)"      OFF)
    option(AME_BUILD_AUTHORING "Build the graphical authoring tool" ON)
    option(AME_BUILD_PYTHON "Build Python bindings for ame_core" OFF)
    if(AME_LLM OR AME_NEURAL)
        set(AME_NEURO ON CACHE BOOL "" FORCE)
    endif()
    if(AME_LLM)
        find_package(CURL REQUIRED)
    endif()

    enable_testing()
endif()

# The PCL component layer and the PYRAMID autonomy bridge are the only parts of
# AME that need those two subprojects, and they cannot be built without them.
# This is not a preference, so it is worked out rather than offered as an
# option: the layer is built exactly when PCL is there to build it against.
if(TARGET pcl_core)
    set(AME_BUILD_PCL_COMPONENTS ON)
else()
    set(AME_BUILD_PCL_COMPONENTS OFF)
    message(STATUS
        "AME: PCL is not part of this build, so the component layer and the "
        "PYRAMID bridge are not built. The core engine, the authoring tool, "
        "ame_mission_cli and the contingency verifier are unaffected.")
endif()

include(AmeDependencies)
ame_add_behaviortree_cpp()
ame_add_lapkt()
ame_add_googletest()
ame_add_nlohmann_json()
if(AME_FOXGLOVE)
    ame_add_websocket_headers()
endif()

add_subdirectory(src)
add_subdirectory(tests)
