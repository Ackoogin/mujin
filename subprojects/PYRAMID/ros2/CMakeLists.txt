cmake_minimum_required(VERSION 3.21)
project(pyramid_ros2_transport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT DEFINED MUJIN_ROOT)
    get_filename_component(MUJIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
endif()
file(TO_CMAKE_PATH "${MUJIN_ROOT}" MUJIN_ROOT)

if(NOT DEFINED PCL_CORE_INCLUDE_DIR)
    set(PCL_CORE_INCLUDE_DIR "${MUJIN_ROOT}/subprojects/PCL/include")
endif()
file(TO_CMAKE_PATH "${PCL_CORE_INCLUDE_DIR}" PCL_CORE_INCLUDE_DIR)

if(NOT DEFINED PCL_CORE_LIB)
    if(WIN32)
        set(PCL_CORE_LIB "${MUJIN_ROOT}/build/subprojects/PCL/src/Release/pcl_core.lib")
    else()
        set(PCL_CORE_LIB "${MUJIN_ROOT}/build/subprojects/PCL/src/libpcl_core.a")
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

add_library(${PROJECT_NAME}_lib SHARED
    src/rclcpp_runtime_adapter.cpp
    ${MUJIN_ROOT}/subprojects/PYRAMID/bindings/cpp/generated/ros2/cpp/pyramid_ros2_transport_support.cpp
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MUJIN_ROOT}/subprojects/PYRAMID/bindings/cpp/generated/ros2/cpp>
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

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_package()
