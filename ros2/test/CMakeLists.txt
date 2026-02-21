find_package(ament_cmake_gtest REQUIRED)

# Helper: link everything a test node needs
function(mujin_ros2_test target src)
    ament_add_gtest(${target} ${src})
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${BTCPP_INCLUDE_DIR}
    )
    ament_target_dependencies(${target}
        rclcpp rclcpp_action rclcpp_lifecycle std_msgs lifecycle_msgs)
    target_link_libraries(${target} mujin_ros2_lib)
    rosidl_get_typesupport_target(ts ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${target} ${ts})
endfunction()

mujin_ros2_test(test_world_model_node test_world_model_node.cpp)
mujin_ros2_test(test_planner_node     test_planner_node.cpp)
mujin_ros2_test(test_executor_node    test_executor_node.cpp)
mujin_ros2_test(test_full_pipeline    test_full_pipeline.cpp)
