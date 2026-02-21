# ---------------------------------------------------------------------------
# mujin_core — ROS-agnostic planning + execution library
# ---------------------------------------------------------------------------
add_library(mujin_core STATIC
    type_system.cpp
    world_model.cpp
    action_registry.cpp
    plan_compiler.cpp
    pddl_parser.cpp
    planner.cpp
    bt_nodes/check_world_predicate.cpp
    bt_nodes/set_world_predicate.cpp
    bt_logger.cpp
    wm_audit_log.cpp
    plan_audit_log.cpp
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
# Foxglove WebSocket bridge (optional)
# ---------------------------------------------------------------------------
if(MUJIN_FOXGLOVE)
    add_library(mujin_foxglove STATIC foxglove_bridge.cpp)

    target_include_directories(mujin_foxglove PUBLIC
        ${CMAKE_SOURCE_DIR}/include
    )
    target_include_directories(mujin_foxglove PRIVATE
        ${websocketpp_SOURCE_DIR}
        ${asio_SOURCE_DIR}/asio/include
    )

    target_link_libraries(mujin_foxglove PUBLIC mujin_core)
    target_compile_definitions(mujin_foxglove PRIVATE
        ASIO_STANDALONE
        _WEBSOCKETPP_CPP11_STL_
    )

    # Suppress warnings from third-party header-only code
    if(MSVC)
        target_compile_options(mujin_foxglove PRIVATE /W0)
    else()
        target_compile_options(mujin_foxglove PRIVATE -w)
    endif()

    find_package(Threads REQUIRED)
    target_link_libraries(mujin_foxglove PRIVATE Threads::Threads)
endif()

# ---------------------------------------------------------------------------
# Demo / vertical-slice executable
# ---------------------------------------------------------------------------
add_executable(mujin_test_app main.cpp)

target_link_libraries(mujin_test_app
    PRIVATE
        mujin_core
)

if(MUJIN_FOXGLOVE)
    target_link_libraries(mujin_test_app PRIVATE mujin_foxglove)
    target_compile_definitions(mujin_test_app PRIVATE MUJIN_FOXGLOVE=1)
endif()

# Copy BT.CPP DLL next to the executable on Windows
if(WIN32)
    add_custom_command(TARGET mujin_test_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:behaviortree_cpp>
            $<TARGET_FILE_DIR:mujin_test_app>
        COMMENT "Copying behaviortree_cpp DLL to output directory"
    )
endif()
