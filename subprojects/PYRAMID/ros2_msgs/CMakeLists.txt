cmake_minimum_required(VERSION 3.21)
project(pyramid_msgs)

# Native ROS2 interface package: turns the generated PYRAMID .proto -> ROS2 IDL
# into real typed messages/services via rosidl. The .msg/.srv are produced by the
# host build's ros2 backend into <bindings>/ros2/idl/{msg,srv} (WIP §2.G.2: no
# committed generated files). rosidl requires interface files relative to the
# package source dir, so we copy them in from the build tree at configure time;
# the copied msg/ and srv/ dirs are git-ignored.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT DEFINED UNMANNED_ROOT)
    get_filename_component(UNMANNED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
endif()
file(TO_CMAKE_PATH "${UNMANNED_ROOT}" UNMANNED_ROOT)

# Where the generated IDL lives (the host build's bindings tree by default).
if(NOT DEFINED PYRAMID_MSGS_IDL_DIR)
    if(DEFINED PYRAMID_CPP_BINDINGS_DIR)
        set(PYRAMID_MSGS_IDL_DIR "${PYRAMID_CPP_BINDINGS_DIR}/ros2/idl")
    else()
        set(PYRAMID_MSGS_IDL_DIR
            "${UNMANNED_ROOT}/build-ros2/generated/pyramid_cpp_bindings/ros2/idl")
    endif()
endif()
file(TO_CMAKE_PATH "${PYRAMID_MSGS_IDL_DIR}" PYRAMID_MSGS_IDL_DIR)

if(NOT EXISTS "${PYRAMID_MSGS_IDL_DIR}/msg")
    message(FATAL_ERROR
        "Generated ROS2 IDL not found at\n  ${PYRAMID_MSGS_IDL_DIR}\n"
        "Build the main workspace with PYRAMID_ENABLE_ROS2=ON first (it generates "
        "the IDL into the build tree), or pass -DPYRAMID_MSGS_IDL_DIR=<dir>.")
endif()

# Copy the generated IDL into the (git-ignored) package source dirs so rosidl can
# resolve each interface relative to CMAKE_CURRENT_SOURCE_DIR.
file(REMOVE_RECURSE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(COPY "${PYRAMID_MSGS_IDL_DIR}/msg" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}")
file(COPY "${PYRAMID_MSGS_IDL_DIR}/srv" DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
list(SORT MSG_FILES)
list(SORT SRV_FILES)

if(NOT MSG_FILES AND NOT SRV_FILES)
    message(FATAL_ERROR "No .msg/.srv found under ${PYRAMID_MSGS_IDL_DIR}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    ${SRV_FILES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
