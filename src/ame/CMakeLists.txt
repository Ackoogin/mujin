# ---------------------------------------------------------------------------
# ame_core — ROS-agnostic planning + execution library
# ---------------------------------------------------------------------------
add_library(ame_core STATIC
    type_system.cpp
    world_model.cpp
    action_registry.cpp
    plan_compiler.cpp
    pddl_parser.cpp
    planner.cpp
    world_model_component.cpp
    planner_component.cpp
    executor_component.cpp
    goal_allocator.cpp
    agent_dispatcher.cpp
    bt_nodes/check_world_predicate.cpp
    bt_nodes/set_world_predicate.cpp
    bt_nodes/invoke_service.cpp
    bt_nodes/execute_phase_action.cpp
    bt_nodes/delegate_to_agent.cpp
    bt_logger.cpp
    wm_audit_log.cpp
    plan_audit_log.cpp
    perception_bridge.cpp
    world_model_snapshot.cpp
)

target_include_directories(ame_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(ame_core
    PUBLIC
        behaviortree_cpp
        lapkt_core
        pcl_core
        pyramid_core
)

# ---------------------------------------------------------------------------
# Foxglove WebSocket bridge (optional)
# ---------------------------------------------------------------------------
if(AME_FOXGLOVE)
    add_library(ame_foxglove STATIC foxglove_bridge.cpp)

    target_include_directories(ame_foxglove PUBLIC
        ${CMAKE_SOURCE_DIR}/include
    )
    target_include_directories(ame_foxglove PRIVATE
        ${websocketpp_SOURCE_DIR}
        ${asio_SOURCE_DIR}/asio/include
    )

    target_link_libraries(ame_foxglove PUBLIC ame_core)
    target_compile_definitions(ame_foxglove PRIVATE
        ASIO_STANDALONE
        _WEBSOCKETPP_CPP11_STL_
    )

    # Suppress warnings from third-party header-only code
    if(MSVC)
        target_compile_options(ame_foxglove PRIVATE /W0)
    else()
        target_compile_options(ame_foxglove PRIVATE -w)
    endif()

    find_package(Threads REQUIRED)
    target_link_libraries(ame_foxglove PRIVATE Threads::Threads)
endif()

# ---------------------------------------------------------------------------
# Demo / vertical-slice executable
# ---------------------------------------------------------------------------
add_executable(ame_test_app main.cpp)

target_link_libraries(ame_test_app
    PRIVATE
        ame_core
)

if(AME_FOXGLOVE)
    target_link_libraries(ame_test_app PRIVATE ame_foxglove)
    target_compile_definitions(ame_test_app PRIVATE AME_FOXGLOVE=1)
endif()

# ---------------------------------------------------------------------------
# Install rules — allows ros2/ ament_cmake package to find_package(ame_core)
#
# We avoid install(EXPORT) because lapkt_core and behaviortree_cpp are
# FetchContent targets with absolute source/build-tree paths in their
# INTERFACE_INCLUDE_DIRECTORIES, which CMake rejects at export time.
# Instead we use a hand-written config file (find_library-based).
#
# behaviortree_cpp is installed via install(FILES $<TARGET_FILE:...>) rather
# than install(TARGETS ...) to avoid triggering the cmake_install.cmake chain
# for a target whose cmake_install.cmake was never generated
# (CMAKE_SKIP_INSTALL_RULES was TRUE during FetchContent_MakeAvailable).
#
# For the same reason we create a stub cmake_install.cmake for behaviortree_cpp
# so that cmake --install does not fail when the root cmake_install.cmake tries
# to include the (otherwise absent) sub-directory script.
# ---------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

# Stub for behaviortree_cpp's cmake_install.cmake (not generated because
# CMAKE_SKIP_INSTALL_RULES was TRUE during FetchContent_MakeAvailable).
if(NOT EXISTS "${behaviortree_cpp_BINARY_DIR}/cmake_install.cmake")
    file(WRITE "${behaviortree_cpp_BINARY_DIR}/cmake_install.cmake"
         "# Stub: behaviortree_cpp install rules are handled by ame install targets.\n")
endif()

# Static archives
install(TARGETS ame_core lapkt_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# behaviortree_cpp static archive — copy the built file directly
install(FILES $<TARGET_FILE:behaviortree_cpp>
    DESTINATION lib
)

# BT.CPP public headers (part of ame_core's API surface via bt_logger.h etc.)
install(DIRECTORY ${behaviortree_cpp_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# ame_core public headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Hand-written config file: creates ame::ame_core IMPORTED target
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/ame_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ame_coreConfig.cmake
    INSTALL_DESTINATION lib/cmake/ame_core
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ame_coreConfig.cmake
    DESTINATION lib/cmake/ame_core
)

# Copy BT.CPP DLL next to the executable on Windows
if(WIN32)
    add_custom_command(TARGET ame_test_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:behaviortree_cpp>
            $<TARGET_FILE_DIR:ame_test_app>
        COMMENT "Copying behaviortree_cpp DLL to output directory"
    )
endif()

# ---------------------------------------------------------------------------
# Python bindings (optional, requires AME_BUILD_PYTHON=ON)
# ---------------------------------------------------------------------------
if(AME_BUILD_PYTHON)
    pybind11_add_module(_ame_py python/ame_py.cpp)
    target_link_libraries(_ame_py PRIVATE ame_core)
    
    # Install Python module to a predictable location
    install(TARGETS _ame_py
        LIBRARY DESTINATION python
        RUNTIME DESTINATION python
    )
    
    # Copy to build directory for easier testing
    add_custom_command(TARGET _ame_py POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:_ame_py>
            ${CMAKE_BINARY_DIR}/python/$<TARGET_FILE_NAME:_ame_py>
        COMMENT "Copying _ame_py to build/python directory"
    )
    
    # Ensure build/python directory exists
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/python)
endif()
