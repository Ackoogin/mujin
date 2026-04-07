find_package(ament_cmake_gtest REQUIRED)

set(ame_ros2_test_path_entries "")
set(ame_ros2_test_pythonpath "")
if(WIN32)
    get_filename_component(ros2_prefix "${rclcpp_DIR}" DIRECTORY)
    get_filename_component(ros2_prefix "${ros2_prefix}" DIRECTORY)
    get_filename_component(ros2_prefix "${ros2_prefix}" DIRECTORY)

    list(APPEND ame_ros2_test_path_entries
        "$<TARGET_FILE_DIR:ame_ros2_lib>"
        "${ros2_prefix}/bin"
        "${ros2_prefix}/Scripts"
    )

    if(DEFINED Python3_EXECUTABLE)
        get_filename_component(python3_dir "${Python3_EXECUTABLE}" DIRECTORY)
        set(pixi_env_prefix "${python3_dir}")
        list(APPEND ame_ros2_test_path_entries
            "${pixi_env_prefix}/bin"
            "${pixi_env_prefix}/Library/bin"
        )
    endif()

    set(ame_ros2_test_pythonpath "${ros2_prefix}/Lib/site-packages")
endif()

# Helper: link everything a test node needs
function(ame_ros2_test target src)
    ament_add_gtest(${target} ${src})
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${BTCPP_INCLUDE_DIR}
    )
    ament_target_dependencies(${target}
        rclcpp rclcpp_action rclcpp_lifecycle std_msgs lifecycle_msgs)
    target_link_libraries(${target} ame_ros2_lib)
    rosidl_get_typesupport_target(ts ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${target} ${ts})
    if(WIN32)
        unset(test_env_mods)
        foreach(path_entry IN LISTS ame_ros2_test_path_entries)
            list(APPEND test_env_mods "PATH=path_list_prepend:${path_entry}")
        endforeach()
        if(ame_ros2_test_pythonpath)
            list(APPEND test_env_mods
                "PYTHONPATH=path_list_prepend:${ame_ros2_test_pythonpath}"
            )
        endif()
        set_tests_properties(${target} PROPERTIES
            ENVIRONMENT_MODIFICATION "${test_env_mods}"
        )
    endif()
endfunction()

ame_ros2_test(test_world_model_node   test_world_model_node.cpp)
ame_ros2_test(test_planner_node       test_planner_node.cpp)
ame_ros2_test(test_executor_node      test_executor_node.cpp)
ame_ros2_test(test_full_pipeline      test_full_pipeline.cpp)
ame_ros2_test(test_extension_wiring   test_extension_wiring.cpp)
ame_ros2_test(test_e2e_pyramid_service test_e2e_pyramid_service.cpp)
