# ---------------------------------------------------------------------------
# mujin_core — ROS-agnostic planning + execution library
# ---------------------------------------------------------------------------
add_library(mujin_core STATIC
    type_system.cpp
    world_model.cpp
    action_registry.cpp
    plan_compiler.cpp
    bt_nodes/check_world_predicate.cpp
    bt_nodes/set_world_predicate.cpp
)

target_include_directories(mujin_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(mujin_core
    PUBLIC
        behaviortree_cpp
        lapkt_core
)

# ---------------------------------------------------------------------------
# Demo / vertical-slice executable
# ---------------------------------------------------------------------------
add_executable(mujin_test_app main.cpp)

target_link_libraries(mujin_test_app
    PRIVATE
        mujin_core
)

# Copy BT.CPP DLL next to the executable on Windows
if(WIN32)
    add_custom_command(TARGET mujin_test_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:behaviortree_cpp>
            $<TARGET_FILE_DIR:mujin_test_app>
        COMMENT "Copying behaviortree_cpp DLL to output directory"
    )
endif()
