find_package(ament_cmake_gtest REQUIRED)

set(pyramid_ros2_test_path_entries "")
set(pyramid_ros2_test_pythonpath "")
if(WIN32)
    get_filename_component(ros2_prefix "${rclcpp_DIR}" DIRECTORY)
    get_filename_component(ros2_prefix "${ros2_prefix}" DIRECTORY)
    get_filename_component(ros2_prefix "${ros2_prefix}" DIRECTORY)

    list(APPEND pyramid_ros2_test_path_entries
        "$<TARGET_FILE_DIR:${PROJECT_NAME}_lib>"
        "${ros2_prefix}/bin"
        "${ros2_prefix}/Scripts"
    )

    if(DEFINED Python3_EXECUTABLE)
        get_filename_component(python3_dir "${Python3_EXECUTABLE}" DIRECTORY)
        list(APPEND pyramid_ros2_test_path_entries
            "${python3_dir}/bin"
            "${python3_dir}/Library/bin"
        )
    endif()

    set(pyramid_ros2_test_pythonpath "${ros2_prefix}/Lib/site-packages")
endif()

ament_add_gtest(test_rclcpp_runtime_adapter test_rclcpp_runtime_adapter.cpp)
target_include_directories(test_rclcpp_runtime_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${MUJIN_ROOT}/subprojects/PYRAMID/bindings/cpp/generated/ros2/cpp
)
ament_target_dependencies(test_rclcpp_runtime_adapter rclcpp)
target_link_libraries(test_rclcpp_runtime_adapter
    ${PROJECT_NAME}_lib
    pcl_core_external
)
rosidl_get_typesupport_target(ts ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_rclcpp_runtime_adapter ${ts})

if(WIN32)
    unset(test_env_mods)
    foreach(path_entry IN LISTS pyramid_ros2_test_path_entries)
        list(APPEND test_env_mods "PATH=path_list_prepend:${path_entry}")
    endforeach()
    if(pyramid_ros2_test_pythonpath)
        list(APPEND test_env_mods
            "PYTHONPATH=path_list_prepend:${pyramid_ros2_test_pythonpath}"
        )
    endif()
    set_tests_properties(test_rclcpp_runtime_adapter PROPERTIES
        ENVIRONMENT_MODIFICATION "${test_env_mods}"
    )
endif()
