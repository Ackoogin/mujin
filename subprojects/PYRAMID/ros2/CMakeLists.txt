cmake_minimum_required(VERSION 3.21)
project(pyramid_ros2_transport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT DEFINED UNMANNED_ROOT)
    get_filename_component(UNMANNED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
endif()
file(TO_CMAKE_PATH "${UNMANNED_ROOT}" UNMANNED_ROOT)

if(NOT DEFINED PCL_CORE_INCLUDE_DIR)
    set(PCL_CORE_INCLUDE_DIR "${UNMANNED_ROOT}/subprojects/PCL/include")
endif()
file(TO_CMAKE_PATH "${PCL_CORE_INCLUDE_DIR}" PCL_CORE_INCLUDE_DIR)

# Generated C++ bindings (incl. the ROS2 transport support layer) are produced at
# build time, not committed (WIP §2.G.2). The host workspace generates them into
# its build tree under generated/pyramid_cpp_bindings; build_ros2_transport.sh
# passes that path as PYRAMID_CPP_BINDINGS_DIR so colcon compiles the freshly
# generated support with no committed generated files.
if(NOT DEFINED PYRAMID_CPP_BINDINGS_DIR)
    set(PYRAMID_CPP_BINDINGS_DIR
        "${UNMANNED_ROOT}/build-ros2/generated/pyramid_cpp_bindings")
endif()
file(TO_CMAKE_PATH "${PYRAMID_CPP_BINDINGS_DIR}" PYRAMID_CPP_BINDINGS_DIR)
set(PYRAMID_ROS2_SUPPORT_DIR "${PYRAMID_CPP_BINDINGS_DIR}/ros2/cpp")

if(NOT EXISTS "${PYRAMID_ROS2_SUPPORT_DIR}/pyramid_ros2_transport_support.cpp")
    message(FATAL_ERROR
        "Generated ROS2 transport support not found at\n  ${PYRAMID_ROS2_SUPPORT_DIR}\n"
        "Build the main workspace with PYRAMID_ENABLE_ROS2=ON first (it generates "
        "into the build tree), or pass -DPYRAMID_CPP_BINDINGS_DIR=<dir>.")
endif()

if(NOT DEFINED PCL_CORE_LIB)
    if(WIN32)
        set(PCL_CORE_LIB "${UNMANNED_ROOT}/build/subprojects/PCL/src/Release/pcl_core.lib")
    else()
        set(PCL_CORE_LIB "${UNMANNED_ROOT}/build/subprojects/PCL/src/libpcl_core.a")
    endif()
endif()
file(TO_CMAKE_PATH "${PCL_CORE_LIB}" PCL_CORE_LIB)

if(NOT EXISTS "${PCL_CORE_LIB}")
    message(FATAL_ERROR
        "PCL core library not found at ${PCL_CORE_LIB}. Build the main workspace first.")
endif()

add_library(pcl_core_external STATIC IMPORTED GLOBAL)
set_target_properties(pcl_core_external PROPERTIES
    IMPORTED_LOCATION "${PCL_CORE_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${PCL_CORE_INCLUDE_DIR}"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PclEnvelope.msg"
    "srv/PclService.srv"
    "srv/PclOpenStream.srv"
)

# Compile the generic support layer plus every generated per-component facade
# (ServiceBinder) found alongside it, so ROS2 ingress can be bound by name.
file(GLOB PYRAMID_ROS2_SUPPORT_SRCS CONFIGURE_DEPENDS
    "${PYRAMID_ROS2_SUPPORT_DIR}/*.cpp")

add_library(${PROJECT_NAME}_lib SHARED
    src/rclcpp_runtime_adapter.cpp
    ${PYRAMID_ROS2_SUPPORT_SRCS}
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PYRAMID_ROS2_SUPPORT_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp)
target_link_libraries(${PROJECT_NAME}_lib pcl_core_external)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})

install(TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------------------------
# pyramid_ros2_coupled_plugin -- coupled ROS2 target plugin (application/ros2)
# ---------------------------------------------------------------------------
# Mirrors the coupled gRPC target: a single MODULE .so exporting both a PCL
# transport vtable and a PCL codec vtable under application/ros2. All ROS2
# linkage (rclcpp, the RclcppRuntimeAdapter, the generated support layer, the
# PclEnvelope/PclService/PclOpenStream typesupport) is bundled privately into
# the .so, so a consumer links only pcl_core and dlopen()s the plugin.
# ---------------------------------------------------------------------------
add_library(pyramid_ros2_coupled_plugin MODULE
    ${UNMANNED_ROOT}/subprojects/PYRAMID/plugins/pyramid_ros2_coupled_plugin.cpp
)
set_target_properties(pyramid_ros2_coupled_plugin PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    CXX_VISIBILITY_PRESET hidden
)
target_include_directories(pyramid_ros2_coupled_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PYRAMID_ROS2_SUPPORT_DIR}
)
ament_target_dependencies(pyramid_ros2_coupled_plugin rclcpp)
target_link_libraries(pyramid_ros2_coupled_plugin
    ${PROJECT_NAME}_lib
    pcl_core_external
    ${cpp_typesupport_target}
)

install(TARGETS pyramid_ros2_coupled_plugin
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_package()
