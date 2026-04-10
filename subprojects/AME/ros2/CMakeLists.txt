cmake_minimum_required(VERSION 3.21)
project(ame_ros2)

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
# ame_core: installed by the plain CMake build.
# Set AME_CORE_INSTALL_PREFIX to override (default: sibling build/install).
# ---------------------------------------------------------------------------
if(NOT DEFINED AME_CORE_INSTALL_PREFIX)
    set(AME_CORE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/install")
endif()

find_package(ame_core REQUIRED
    HINTS "${AME_CORE_INSTALL_PREFIX}/lib/cmake/ame_core"
)
# ame::ame_core's INTERFACE_INCLUDE_DIRECTORIES already contains the install
# prefix include/ directory, which holds both ame and BT.CPP headers.

# ---------------------------------------------------------------------------
# Custom message / service / action generation
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/WorldFact.msg"
    "msg/WorldState.msg"
    "msg/Detection.msg"
    "srv/GetFact.srv"
    "srv/SetFact.srv"
    "srv/QueryState.srv"
    "srv/DispatchGoals.srv"
    "srv/LoadDomain.srv"
    "srv/LoadBt.srv"
    "srv/StopExecution.srv"
    "action/Plan.action"
    DEPENDENCIES std_msgs
)

# ---------------------------------------------------------------------------
# ame_ros2_lib — the three node classes + RosWmBridge + LifecycleManager
# ---------------------------------------------------------------------------
add_library(ame_ros2_lib SHARED
    src/nodes/world_model_node.cpp
    src/nodes/planner_node.cpp
    src/nodes/executor_node.cpp
    src/nodes/agent_dispatcher_node.cpp
    src/lib/ros_wm_bridge.cpp
    src/lib/lifecycle_manager.cpp
)
# On Windows, export all symbols so the import library (.lib) is generated.
set_target_properties(ame_ros2_lib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

target_include_directories(ame_ros2_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ame_ros2_lib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    lifecycle_msgs
    ament_index_cpp
)

target_link_libraries(ame_ros2_lib
    ame::ame_core
)

# Link generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ame_ros2_lib ${cpp_typesupport_target})

# ---------------------------------------------------------------------------
# Standalone node executables (distributed deployment)
# ---------------------------------------------------------------------------
add_executable(world_model_node src/apps/world_model_node_main.cpp)
target_link_libraries(world_model_node ame_ros2_lib)

add_executable(planner_node src/apps/planner_node_main.cpp)
target_link_libraries(planner_node ame_ros2_lib)

add_executable(executor_node src/apps/executor_node_main.cpp)
target_link_libraries(executor_node ame_ros2_lib)

add_executable(agent_dispatcher_node src/apps/agent_dispatcher_node_main.cpp)
target_link_libraries(agent_dispatcher_node ame_ros2_lib)

# ---------------------------------------------------------------------------
# Combined in-process executable (all three nodes on one executor)
# ---------------------------------------------------------------------------
add_executable(ame_combined src/apps/combined_main.cpp)
target_link_libraries(ame_combined ame_ros2_lib)

if(TARGET ame::ame_foxglove)
    add_executable(foxglove_bridge src/apps/foxglove_bridge_main.cpp)
    target_link_libraries(foxglove_bridge
        ame_ros2_lib
        ame::ame_foxglove
    )
endif()

# ---------------------------------------------------------------------------
# Lifecycle manager standalone executable (for distributed launch)
# ---------------------------------------------------------------------------
add_executable(lifecycle_manager src/apps/lifecycle_manager_main.cpp)
target_link_libraries(lifecycle_manager ame_ros2_lib)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS ame_ros2_lib world_model_node planner_node executor_node agent_dispatcher_node ame_combined lifecycle_manager
    EXPORT ame_ros2_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(TARGET foxglove_bridge)
    install(TARGETS foxglove_bridge
        EXPORT ame_ros2_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
endif()

ament_export_targets(ame_ros2_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rclcpp_lifecycle std_msgs lifecycle_msgs)
ament_package()
