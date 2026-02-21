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

# ---------------------------------------------------------------------------
# Install rules — allows ros2/ ament_cmake package to find_package(mujin_core)
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
         "# Stub: behaviortree_cpp install rules are handled by mujin install targets.\n")
endif()

# Static archives
install(TARGETS mujin_core lapkt_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# behaviortree_cpp static archive — copy the built file directly
install(FILES $<TARGET_FILE:behaviortree_cpp>
    DESTINATION lib
)

# BT.CPP public headers (part of mujin_core's API surface via bt_logger.h etc.)
install(DIRECTORY ${behaviortree_cpp_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# mujin_core public headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Hand-written config file: creates mujin::mujin_core IMPORTED target
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/mujin_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/mujin_coreConfig.cmake
    INSTALL_DESTINATION lib/cmake/mujin_core
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/mujin_coreConfig.cmake
    DESTINATION lib/cmake/mujin_core
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
