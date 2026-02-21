cmake_minimum_required(VERSION 3.21)
project(mujin_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# ROS2 dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------------------
# mujin_core: installed by the plain CMake build.
# Set MUJIN_CORE_INSTALL_PREFIX to override (default: sibling build/install).
# ---------------------------------------------------------------------------
if(NOT DEFINED MUJIN_CORE_INSTALL_PREFIX)
    set(MUJIN_CORE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../build/install")
endif()

find_package(mujin_core REQUIRED
    HINTS "${MUJIN_CORE_INSTALL_PREFIX}/lib/cmake/mujin_core"
)
# mujin::mujin_core's INTERFACE_INCLUDE_DIRECTORIES already contains the install
# prefix include/ directory, which holds both mujin and BT.CPP headers.

# ---------------------------------------------------------------------------
# Custom message / service / action generation
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/WorldFact.msg"
    "msg/WorldState.msg"
    "srv/GetFact.srv"
    "srv/SetFact.srv"
    "srv/QueryState.srv"
    "action/Plan.action"
    DEPENDENCIES std_msgs
)

# ---------------------------------------------------------------------------
# mujin_ros2_lib — the three node classes + RosWmBridge + LifecycleManager
# ---------------------------------------------------------------------------
add_library(mujin_ros2_lib SHARED
    src/world_model_node.cpp
    src/planner_node.cpp
    src/executor_node.cpp
    src/ros_wm_bridge.cpp
    src/lifecycle_manager.cpp
)

target_include_directories(mujin_ros2_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mujin_ros2_lib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    lifecycle_msgs
)

target_link_libraries(mujin_ros2_lib
    mujin::mujin_core
)

# Link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mujin_ros2_lib ${cpp_typesupport_target})

# ---------------------------------------------------------------------------
# Standalone node executables (distributed deployment)
# ---------------------------------------------------------------------------
add_executable(world_model_node src/world_model_node_main.cpp)
target_link_libraries(world_model_node mujin_ros2_lib)

add_executable(planner_node src/planner_node_main.cpp)
target_link_libraries(planner_node mujin_ros2_lib)

add_executable(executor_node src/executor_node_main.cpp)
target_link_libraries(executor_node mujin_ros2_lib)

# ---------------------------------------------------------------------------
# Combined in-process executable (all three nodes on one executor)
# ---------------------------------------------------------------------------
add_executable(mujin_combined src/combined_main.cpp)
target_link_libraries(mujin_combined mujin_ros2_lib)

# ---------------------------------------------------------------------------
# Lifecycle manager standalone executable (for distributed launch)
# ---------------------------------------------------------------------------
add_executable(lifecycle_manager src/lifecycle_manager_main.cpp)
target_link_libraries(lifecycle_manager mujin_ros2_lib)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS mujin_ros2_lib world_model_node planner_node executor_node mujin_combined lifecycle_manager
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
endif()

ament_export_targets(mujin_ros2_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rclcpp_lifecycle std_msgs lifecycle_msgs)
ament_package()
