# ---------------------------------------------------------------------------
# ame_core -- ROS-agnostic planning + execution library
# ---------------------------------------------------------------------------
set(_AME_CORE_SOURCES
    lib/type_system.cpp
    lib/world_model.cpp
    lib/world_state_access.cpp
    lib/action_registry.cpp
    lib/plan_compiler.cpp
    lib/pddl_parser.cpp
    lib/planner.cpp
    lib/contingency_search.cpp
    lib/goal_allocator.cpp
    lib/execution_sink.cpp
    nodes/check_world_predicate.cpp
    nodes/planned_action_node.cpp
    nodes/planned_action.cpp
    nodes/simulated_action.cpp
    nodes/goal_reached.cpp
    nodes/ame_dispatch_node.cpp
    nodes/invoke_service.cpp
    nodes/execute_phase_action.cpp
    nodes/delegate_to_agent.cpp
    nodes/follow_route.cpp
    lib/spatial_oracle.cpp
    lib/bt_logger.cpp
    lib/wm_audit_log.cpp
    lib/plan_audit_log.cpp
    lib/perception_bridge.cpp
    lib/world_model_snapshot.cpp
)

set(_AME_PCL_COMPONENT_SOURCES
    lib/pcl_msg_json.cpp
    lib/world_model_component.cpp
    lib/planner_component.cpp
    lib/executor_component.cpp
    lib/agent_dispatcher.cpp
    # autonomy_backend implements CurrentAmeBackendAdapter, which embeds an
    # ExecutorComponent; it is a component-layer adapter, not pure core.
    lib/autonomy_backend.cpp
)

# pyramid_autonomy_bridge.cpp includes a generated PYRAMID header; only
# compile it when the PYRAMID subproject is included in the build.
if(UNMANNED_BUILD_PYRAMID)
    list(APPEND _AME_PCL_COMPONENT_SOURCES lib/pyramid_autonomy_bridge.cpp)
    if(AME_BUILD_AGRA_MA_BRIDGE)
        list(APPEND _AME_PCL_COMPONENT_SOURCES
            lib/agra_ma_bridge.cpp)
    endif()
endif()

add_library(ame_core STATIC ${_AME_CORE_SOURCES})

target_include_directories(ame_core PUBLIC
    ${AME_ROOT_DIR}/include
)
target_include_directories(ame_core PRIVATE
    $<TARGET_PROPERTY:pcl_core,INTERFACE_INCLUDE_DIRECTORIES>
)

# PYRAMID owns the C++ convenience layer over PCL's C ABI. AME needs those
# headers even when the rest of the PYRAMID subproject is disabled.
if(NOT TARGET pyramid_pcl_cpp)
    add_library(pyramid_pcl_cpp INTERFACE)
    target_include_directories(pyramid_pcl_cpp INTERFACE
        ${PYRAMID_ROOT_DIR}/include
    )
    target_link_libraries(pyramid_pcl_cpp INTERFACE pcl_core)
endif()

target_link_libraries(ame_core
    PUBLIC
        behaviortree_cpp
        lapkt_core
)

add_library(ame_pcl_components STATIC ${_AME_PCL_COMPONENT_SOURCES})

target_include_directories(ame_pcl_components PUBLIC
    ${AME_ROOT_DIR}/include
    ${PYRAMID_CPP_BINDINGS_DIR}
)

target_link_libraries(ame_pcl_components
    PUBLIC
        ame_core
        pcl_core
        pyramid_pcl_cpp
)

if(UNMANNED_BUILD_PYRAMID)
    add_dependencies(ame_pcl_components pyramid_cpp_bindings_codegen)
    target_link_libraries(ame_pcl_components PUBLIC pyramid_generated_codecs)
    if(AME_BUILD_AGRA_MA_BRIDGE)
        if(NOT TARGET pyramid_agra_profile_data_model)
            message(FATAL_ERROR
                "AME_BUILD_AGRA_MA_BRIDGE requires "
                "PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR to point at checked-in "
                "A-GRA data-model bindings")
        endif()
        # Only the C2 services/components layer is taken from the P3 bindings;
        # the data-model *codec* comes from the consumer's contract via
        # pyramid_agra_profile_data_model.
        #
        # Both directories contain a pyramid_data_model_agra_types.hpp, and
        # only the P3 one may be reachable by that name. Putting the contract
        # directory first does not work and must not be attempted: `#pragma
        # once` keys on the file, so with both reachable a single translation
        # unit includes each of them once and every enum is defined twice.
        # Tried 2026-07-27; ame_py.cpp fails with ~100 "multiple definition of
        # enum class" errors.
        #
        # So the bridge compiles against P3's declarations while linking a
        # codec built from the contract's. That is safe only while the two
        # agree, and they do -- of 1113 structs common to both, zero differ,
        # which is what a strict text slice of one proto should give. It is
        # guarded rather than assumed: see the header-agreement check in
        # AutoMTK's tests/automtk_agra. Without that guard a divergence would
        # be silent, because the type names are identical and it would build
        # and link either way.
        add_dependencies(ame_pcl_components
            pyramid_agra_p3_bindings_codegen)
        target_link_libraries(ame_pcl_components PUBLIC
            pyramid_agra_profile_data_model)
        target_include_directories(ame_pcl_components PUBLIC
            ${PYRAMID_AGRA_P3_BINDINGS_DIR})
        target_compile_definitions(ame_pcl_components PUBLIC
            AME_BUILD_AGRA_MA_BRIDGE=1)
    endif()
endif()

# ---------------------------------------------------------------------------
# Foxglove WebSocket bridge (optional)
# ---------------------------------------------------------------------------
if(AME_FOXGLOVE)
    add_library(ame_foxglove STATIC lib/foxglove_bridge.cpp)

    target_include_directories(ame_foxglove PUBLIC
        ${AME_ROOT_DIR}/include
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

    install(TARGETS ame_foxglove
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()

# ---------------------------------------------------------------------------
# Neuro-symbolic core infrastructure (optional)
# ---------------------------------------------------------------------------
if(AME_NEURO)
    # Propagate the AME_NEURO flag into ame_core so the thin seam code in
    # planner.cpp and executor_component.cpp is compiled in.  This adds no
    # neural dependencies to ame_core — just null-object hook interfaces.
    target_compile_definitions(ame_core PUBLIC AME_NEURO=1)

    add_library(ame_neuro STATIC
        lib/neuro/backend_executor.cpp
        lib/neuro/backend_registry.cpp
        lib/neuro/mock_backend.cpp
        lib/neuro/fallback_policy.cpp
        lib/neuro/grounded_fluent_verifier.cpp
        lib/neuro/forward_sim_verifier.cpp
        lib/neuro/neuro_audit_log.cpp
        lib/neuro/audit_index.cpp
        lib/neuro/neuro_config.cpp
    )

    target_include_directories(ame_neuro PUBLIC ${AME_ROOT_DIR}/include)
    target_link_libraries(ame_neuro PUBLIC ame_core)
    target_compile_definitions(ame_neuro PUBLIC AME_NEURO=1)

    if(MSVC)
        target_compile_options(ame_neuro PRIVATE /W3)
    else()
        target_compile_options(ame_neuro PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    install(TARGETS ame_neuro
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()

# ---------------------------------------------------------------------------
# Demo / vertical-slice executable
# ---------------------------------------------------------------------------
add_executable(ame_test_app apps/main.cpp)

# ---------------------------------------------------------------------------
# Pure-PCL entry point (no ROS2 dependency)
# ---------------------------------------------------------------------------
add_executable(ame_pcl_main apps/ame_pcl_main.cpp)
target_link_libraries(ame_pcl_main PRIVATE ame_pcl_components)
if(AME_FOXGLOVE)
    target_link_libraries(ame_pcl_main PRIVATE ame_foxglove)
    target_compile_definitions(ame_pcl_main PRIVATE AME_FOXGLOVE=1)
endif()

target_link_libraries(ame_test_app
    PRIVATE
        ame_core
)

if(AME_FOXGLOVE)
    target_link_libraries(ame_test_app PRIVATE ame_foxglove)
    target_compile_definitions(ame_test_app PRIVATE AME_FOXGLOVE=1)
endif()
if(AME_NEURO)
    target_link_libraries(ame_test_app PRIVATE ame_neuro)
    target_compile_definitions(ame_test_app PRIVATE AME_NEURO=1)
endif()

# ---------------------------------------------------------------------------
# Contingency verifier — exhaustive safe-state reachability tool
# ---------------------------------------------------------------------------
add_executable(contingency_verifier apps/contingency_verifier.cpp)
target_link_libraries(contingency_verifier PRIVATE ame_core)
target_include_directories(contingency_verifier PRIVATE
    "${PYRAMID_ROOT_DIR}/core/external"
)

# ---------------------------------------------------------------------------
# Install rules -- allows ros2/ ament_cmake package to find_package(ame_core)
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
install(TARGETS ame_core ame_pcl_components lapkt_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# behaviortree_cpp static archive -- copy the built file directly
install(FILES $<TARGET_FILE:behaviortree_cpp>
    DESTINATION lib
)

# BT.CPP public headers (part of ame_core's API surface via bt_logger.h etc.)
install(DIRECTORY ${behaviortree_cpp_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# ame_core public headers
install(DIRECTORY ${AME_ROOT_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Hand-written config file: creates ame::ame_core IMPORTED target
configure_package_config_file(
    ${UNMANNED_WORKSPACE_ROOT}/cmake/ame_coreConfig.cmake.in
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
    pybind11_add_module(_ame_py bindings/ame_py.cpp)
    target_link_libraries(_ame_py PRIVATE ame_pcl_components)
    if(AME_BUILD_AGRA_MA_BRIDGE)
        set(_AME_PY_AGRA_PORT_SOURCES
            ${PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR}/pyramid_services_agra_c2_provided.cpp
            ${PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR}/pyramid_services_agra_c2_consumed.cpp
            ${PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR}/pyramid_components_agra_c2_services_provided_codec.cpp
            ${PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR}/pyramid_components_agra_c2_services_provided_cabi_marshal.cpp
            ${PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR}/pyramid_components_agra_c2_services_consumed_codec.cpp
            ${PYRAMID_AGRA_OMS_JSON_BINDINGS_DIR}/pyramid_components_agra_c2_services_consumed_cabi_marshal.cpp)
        set_source_files_properties(
            ${_AME_PY_AGRA_PORT_SOURCES}
            PROPERTIES GENERATED TRUE)
        target_sources(_ame_py PRIVATE
            ${_AME_PY_AGRA_PORT_SOURCES}
            ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_agra_ma_grounding_provided.cpp)
        add_dependencies(_ame_py
            pyramid_cpp_bindings_codegen
            pyramid_codec_json_agra_ma_grounding)
        set_source_files_properties(
            ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_agra_ma_grounding_provided.cpp
            PROPERTIES COMPILE_OPTIONS
            "-include;${PYRAMID_CPP_BINDINGS_DIR}/pyramid_components_agra_ma_grounding_services_provided_types.hpp")
        target_compile_definitions(_ame_py PRIVATE
            PYRAMID_AGRA_OMS_JSON_CODEC_PATH="$<TARGET_FILE:pyramid_codec_oms_json_agra_profile>"
            AGRA_MA_GROUNDING_JSON_CODEC_PLUGIN_PATH="$<TARGET_FILE:pyramid_codec_json_agra_ma_grounding>")
    endif()
    
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

option(AME_BUILD_AUTHORING "Build the graphical AME authoring tool (requires SDL2/OpenGL)" OFF)
if(AME_BUILD_AUTHORING)
    add_subdirectory(authoring)
endif()
