include(GoogleTest)

set(TOBJ_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tactical_objects)
set(PROTO_BINDINGS_DIR ${PYRAMID_CPP_BINDINGS_DIR})

find_program(PYRAMID_PYTHON NAMES python3 python)
if(NOT PYRAMID_PYTHON)
    message(FATAL_ERROR "Python is required for PYRAMID binding generator tests")
endif()

set(PYRAMID_TEST_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PYRAMID_TEST_GENERATED_DIR})

set(_pyramid_cpp_test_codec_plugin_targets)
foreach(_pyramid_codec_target IN ITEMS
        pyramid_codec_json_tactical_objects
        pyramid_codec_flatbuffers_tactical_objects
        pyramid_codec_json_autonomy_backend
        pyramid_codec_flatbuffers_autonomy_backend
        pyramid_codec_json_sensor_data_interpretation
        pyramid_codec_flatbuffers_sensor_data_interpretation)
    if(TARGET ${_pyramid_codec_target})
        list(APPEND _pyramid_cpp_test_codec_plugin_targets
            ${_pyramid_codec_target})
    endif()
endforeach()

list(LENGTH _pyramid_cpp_test_codec_plugin_targets
    _pyramid_cpp_test_codec_plugin_count)
set(_pyramid_codec_plugin_env "")
foreach(_pyramid_codec_target IN LISTS _pyramid_cpp_test_codec_plugin_targets)
    if(_pyramid_codec_plugin_env)
        string(APPEND _pyramid_codec_plugin_env ":")
    endif()
    string(APPEND _pyramid_codec_plugin_env "$<TARGET_FILE:${_pyramid_codec_target}>")
endforeach()
set(_pyramid_codec_paths_header
    "#pragma once\n\n#include <array>\n\n")
string(APPEND _pyramid_codec_paths_header
    "inline constexpr std::array<const char*, ${_pyramid_cpp_test_codec_plugin_count}> kPyramidCodecPluginPaths = {{\n")
foreach(_pyramid_codec_target IN LISTS _pyramid_cpp_test_codec_plugin_targets)
    string(APPEND _pyramid_codec_paths_header
        "    \"$<TARGET_FILE:${_pyramid_codec_target}>\",\n")
endforeach()
string(APPEND _pyramid_codec_paths_header "}};\n\n")

function(pyramid_append_plugin_path_constant out_var constant_name target_name)
    set(_line "inline constexpr const char* ${constant_name} = nullptr;\n")
    if(TARGET ${target_name})
        set(_line
            "inline constexpr const char* ${constant_name} = \"$<TARGET_FILE:${target_name}>\";\n")
    endif()
    set(${out_var} "${${out_var}}${_line}" PARENT_SCOPE)
endfunction()

pyramid_append_plugin_path_constant(_pyramid_codec_paths_header
    kPyramidJsonCodecPluginTactical
    pyramid_codec_json_tactical_objects)
pyramid_append_plugin_path_constant(_pyramid_codec_paths_header
    kPyramidFlatbuffersCodecPluginTactical
    pyramid_codec_flatbuffers_tactical_objects)
pyramid_append_plugin_path_constant(_pyramid_codec_paths_header
    kPyramidJsonCodecPluginAutonomy
    pyramid_codec_json_autonomy_backend)
pyramid_append_plugin_path_constant(_pyramid_codec_paths_header
    kPclSocketTransportPlugin
    pcl_transport_socket_plugin)
pyramid_append_plugin_path_constant(_pyramid_codec_paths_header
    kPyramidGrpcCoupledPlugin
    pyramid_grpc_coupled_plugin)

file(GENERATE
    OUTPUT ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>/pyramid_codec_plugin_test_paths.hpp
    CONTENT "${_pyramid_codec_paths_header}")

add_library(pyramid_cpp_test_codec_plugins OBJECT
    pyramid_cpp_codec_plugin_environment.cpp)
if(_pyramid_cpp_test_codec_plugin_targets)
    add_dependencies(pyramid_cpp_test_codec_plugins
        ${_pyramid_cpp_test_codec_plugin_targets})
endif()
target_link_libraries(pyramid_cpp_test_codec_plugins PUBLIC
    pcl_core
    GTest::gtest)
target_include_directories(pyramid_cpp_test_codec_plugins PUBLIC
    ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>)

if(TARGET pyramid_bridge_evidence_client)
    target_include_directories(pyramid_bridge_evidence_client PRIVATE
        ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>)
    if(_pyramid_cpp_test_codec_plugin_targets)
        add_dependencies(pyramid_bridge_evidence_client
            ${_pyramid_cpp_test_codec_plugin_targets})
    endif()
endif()

add_test(NAME pyramid_binding_generation_dependencies
    COMMAND ${PYRAMID_PYTHON}
            ${CMAKE_CURRENT_SOURCE_DIR}/test_binding_generation_dependencies.py
            --generator ${PYRAMID_ROOT_DIR}/pim/generate_bindings.py
            --proto-dir ${PYRAMID_ROOT_DIR}/proto)

add_executable(test_tobj_types ${TOBJ_TEST_DIR}/Test_Types.cpp)
target_link_libraries(test_tobj_types PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_types DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_object_store ${TOBJ_TEST_DIR}/Test_ObjectStore.cpp)
target_link_libraries(test_tobj_object_store PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_object_store DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_milclass ${TOBJ_TEST_DIR}/Test_MilClassEngine.cpp)
target_link_libraries(test_tobj_milclass PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_milclass DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_spatial ${TOBJ_TEST_DIR}/Test_SpatialIndex.cpp)
target_link_libraries(test_tobj_spatial PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_spatial DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_zone ${TOBJ_TEST_DIR}/Test_ZoneEngine.cpp)
target_link_libraries(test_tobj_zone PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_zone DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_correlation ${TOBJ_TEST_DIR}/Test_CorrelationEngine.cpp)
target_link_libraries(test_tobj_correlation PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_correlation DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_generated_component_stream_handle
    test_pcl_generated_component_stream_handle.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
)
add_dependencies(test_pcl_generated_component_stream_handle
    pyramid_cpp_bindings_codegen)
target_link_libraries(test_pcl_generated_component_stream_handle PRIVATE
    pcl_core
    pyramid_generated_codecs
    GTest::gtest_main
)
target_include_directories(test_pcl_generated_component_stream_handle PRIVATE
    ${PROTO_BINDINGS_DIR}
)
gtest_discover_tests(test_pcl_generated_component_stream_handle
    DISCOVERY_MODE PRE_TEST)

add_executable(test_plugin_only_fail_closed
    test_plugin_only_fail_closed.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
)
add_dependencies(test_plugin_only_fail_closed
    pyramid_cpp_bindings_codegen)
target_link_libraries(test_plugin_only_fail_closed PRIVATE
    pcl_core
    pyramid_generated_codecs
    GTest::gtest_main
)
target_include_directories(test_plugin_only_fail_closed PRIVATE
    ${PROTO_BINDINGS_DIR}
)
gtest_discover_tests(test_plugin_only_fail_closed
    DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_query ${TOBJ_TEST_DIR}/Test_QueryEngine.cpp)
target_link_libraries(test_tobj_query PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_query DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_relationship ${TOBJ_TEST_DIR}/Test_RelationshipIndex.cpp)
target_link_libraries(test_tobj_relationship PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_relationship DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_component ${TOBJ_TEST_DIR}/Test_TacticalObjectsComponent.cpp)
target_link_libraries(test_tobj_component PRIVATE tactical_objects_component GTest::gtest_main)
target_include_directories(test_tobj_component PRIVATE ${PCL_ROOT_DIR}/src)
gtest_discover_tests(test_tobj_component DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_component_robustness ${TOBJ_TEST_DIR}/Test_TacticalObjectsComponent_Robustness.cpp)
target_link_libraries(test_tobj_component_robustness PRIVATE tactical_objects_component GTest::gtest_main)
gtest_discover_tests(test_tobj_component_robustness DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_component_hlr ${TOBJ_TEST_DIR}/Test_TacticalObjectsComponent_HLR.cpp)
target_link_libraries(test_tobj_component_hlr PRIVATE tactical_objects_component GTest::gtest_main)
gtest_discover_tests(test_tobj_component_hlr DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_codec ${TOBJ_TEST_DIR}/Test_TacticalObjectsCodec.cpp)
target_link_libraries(test_tobj_codec PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_codec DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_interest ${TOBJ_TEST_DIR}/Test_InterestManager.cpp)
target_link_libraries(test_tobj_interest PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_interest DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_history ${TOBJ_TEST_DIR}/Test_TacticalHistory.cpp)
target_link_libraries(test_tobj_history PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_history DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_runtime ${TOBJ_TEST_DIR}/Test_Runtime.cpp)
target_link_libraries(test_tobj_runtime PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_runtime DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_zone_perf ${TOBJ_TEST_DIR}/Test_ZonePerformance.cpp)
target_link_libraries(test_tobj_zone_perf PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_zone_perf DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_streaming_codec ${TOBJ_TEST_DIR}/Test_StreamingCodec.cpp)
target_link_libraries(test_tobj_streaming_codec PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_streaming_codec DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_runtime_streaming ${TOBJ_TEST_DIR}/Test_TacticalObjectsRuntime_Streaming.cpp)
target_link_libraries(test_tobj_runtime_streaming PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_runtime_streaming DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_interest_matching ${TOBJ_TEST_DIR}/Test_InterestManager_Matching.cpp)
target_link_libraries(test_tobj_interest_matching PRIVATE tactical_objects GTest::gtest_main)
gtest_discover_tests(test_tobj_interest_matching DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_e2e ${TOBJ_TEST_DIR}/Test_TacticalObjects_E2E.cpp)
target_link_libraries(test_tobj_e2e PRIVATE tactical_objects tactical_objects_component pcl_core GTest::gtest_main)
target_include_directories(test_tobj_e2e PRIVATE ${PCL_ROOT_DIR}/src)
gtest_discover_tests(test_tobj_e2e DISCOVERY_MODE PRE_TEST)

add_executable(test_tobj_socket_e2e ${TOBJ_TEST_DIR}/Test_TacticalObjects_SocketE2E.cpp)
target_link_libraries(test_tobj_socket_e2e PRIVATE
    tactical_objects tactical_objects_component
    pcl_core pcl_transport_socket GTest::gtest_main
)
target_include_directories(test_tobj_socket_e2e PRIVATE ${PCL_ROOT_DIR}/src)
if(NOT WIN32)
    target_link_libraries(test_tobj_socket_e2e PRIVATE pthread)
endif()
gtest_discover_tests(test_tobj_socket_e2e DISCOVERY_MODE PRE_TEST)

if(PYRAMID_ENABLE_PROTOBUF)
    add_executable(test_pcl_proto_bindings
        test_pcl_proto_bindings.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
    )
    target_link_libraries(test_pcl_proto_bindings PRIVATE
        pcl_core pcl_transport_socket tactical_objects
        pyramid_generated_codecs
        pyramid_flatbuffers_support
        GTest::gtest_main
    )
    target_include_directories(test_pcl_proto_bindings PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/tactical_objects/include
        ${PYRAMID_ROOT_DIR}/core/include
    )
    if(NOT WIN32)
        target_link_libraries(test_pcl_proto_bindings PRIVATE pthread)
    endif()
    gtest_discover_tests(test_pcl_proto_bindings DISCOVERY_MODE PRE_TEST)
endif()

# ---------------------------------------------------------------------------
# Binding performance benchmark
# ---------------------------------------------------------------------------
if(PYRAMID_ENABLE_FLATBUFFERS)
    add_executable(test_sensor_data_interpretation_bindings
        test_sensor_data_interpretation_bindings.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_sensor_data_interpretation_provided.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_sensor_data_interpretation_consumed.cpp
    )
    target_link_libraries(test_sensor_data_interpretation_bindings PRIVATE
        pcl_core
        pyramid_generated_codecs
        pyramid_flatbuffers_support
        GTest::gtest_main
    )
    target_include_directories(test_sensor_data_interpretation_bindings PRIVATE
        ${PROTO_BINDINGS_DIR}
    )
    if(NOT WIN32)
        target_link_libraries(test_sensor_data_interpretation_bindings PRIVATE pthread)
    endif()
    gtest_discover_tests(test_sensor_data_interpretation_bindings DISCOVERY_MODE PRE_TEST)

    add_executable(test_binding_performance
        test_binding_performance.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
    )
    target_link_libraries(test_binding_performance PRIVATE
        pcl_core pcl_transport_socket pcl_transport_shared_memory
        tactical_objects
        pyramid_generated_codecs
        pyramid_flatbuffers_support
        GTest::gtest_main
    )
    target_include_directories(test_binding_performance PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/tactical_objects/include
        ${PYRAMID_ROOT_DIR}/core/include
    )
    if(PYRAMID_ENABLE_PROTOBUF)
        target_compile_definitions(test_binding_performance PRIVATE PYRAMID_HAS_PROTOBUF)
        target_include_directories(test_binding_performance PRIVATE
            ${PYRAMID_PROTOBUF_GEN_DIR}
        )
    endif()
    if(TARGET pyramid_grpc_transport)
        target_link_libraries(test_binding_performance PRIVATE pyramid_grpc_transport)
        target_include_directories(test_binding_performance PRIVATE
            ${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp
            ${PYRAMID_PROTOBUF_GEN_DIR}
        )
        target_compile_definitions(test_binding_performance PRIVATE PYRAMID_HAS_GRPC)
    endif()
    if(NOT WIN32)
        target_link_libraries(test_binding_performance PRIVATE pthread rt)
    endif()
    gtest_discover_tests(test_binding_performance
        DISCOVERY_MODE PRE_TEST
        PROPERTIES
            LABELS "performance;binding;codec"
            TIMEOUT 120)
endif()

if(PYRAMID_ENABLE_PROTOBUF)
    add_executable(test_codec_dispatch_e2e
        test_codec_dispatch_e2e.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
    )
    target_link_libraries(test_codec_dispatch_e2e PRIVATE
        pcl_core pcl_transport_socket tactical_objects
        pyramid_generated_codecs
        pyramid_flatbuffers_support
        GTest::gtest_main
    )
    target_include_directories(test_codec_dispatch_e2e PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_ROOT_DIR}/tactical_objects/include
        ${PYRAMID_ROOT_DIR}/core/include
    )
    if(NOT WIN32)
        target_link_libraries(test_codec_dispatch_e2e PRIVATE pthread)
    endif()
    gtest_discover_tests(test_codec_dispatch_e2e DISCOVERY_MODE PRE_TEST)
endif()

if(TARGET pyramid_codec_json_tactical_objects)
    add_executable(test_codec_plugin_swap
        test_codec_plugin_swap.cpp
    )
    add_dependencies(test_codec_plugin_swap
        pyramid_codec_json_tactical_objects)
    target_link_libraries(test_codec_plugin_swap PRIVATE
        pcl_core
        pyramid_generated_codecs
        GTest::gtest_main
    )
    target_include_directories(test_codec_plugin_swap PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>
        ${PYRAMID_ROOT_DIR}/tactical_objects/include
        ${PYRAMID_ROOT_DIR}/core/include
    )
    if(PYRAMID_ENABLE_FLATBUFFERS AND TARGET pyramid_codec_flatbuffers_tactical_objects)
        add_dependencies(test_codec_plugin_swap
            pyramid_codec_flatbuffers_tactical_objects)
    endif()
    gtest_discover_tests(test_codec_plugin_swap DISCOVERY_MODE PRE_TEST)
endif()

if(PYRAMID_ENABLE_FLATBUFFERS
        AND TARGET pyramid_codec_flatbuffers_tactical_objects
        AND TARGET pcl_transport_socket_plugin)
    add_executable(test_transport_codec_plugin_composition
        test_transport_codec_plugin_composition.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
    )
    add_dependencies(test_transport_codec_plugin_composition
        pyramid_cpp_bindings_codegen
        pyramid_codec_flatbuffers_tactical_objects
        pcl_transport_socket_plugin)
    target_link_libraries(test_transport_codec_plugin_composition PRIVATE
        pcl_core
        pyramid_generated_codecs
        pyramid_flatbuffers_support
        GTest::gtest_main
    )
    target_include_directories(test_transport_codec_plugin_composition PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>
        ${PYRAMID_ROOT_DIR}/tactical_objects/include
        ${PYRAMID_ROOT_DIR}/core/include
    )
    if(NOT WIN32)
        target_link_libraries(test_transport_codec_plugin_composition PRIVATE pthread)
    endif()
    gtest_discover_tests(test_transport_codec_plugin_composition
        DISCOVERY_MODE PRE_TEST
        PROPERTIES
            LABELS "integration;codec;transport;plugin"
            RESOURCE_LOCK pyramid_process_e2e
            TIMEOUT 15)
endif()

# Bridge scenario: a single process loads codec plugins for two distinct
# components (both under application/json) and dispatches by schema_id.
if(TARGET pyramid_codec_json_tactical_objects
        AND TARGET pyramid_codec_json_autonomy_backend)
    add_executable(test_codec_registry_bridge
        test_codec_registry_bridge.cpp
    )
    add_dependencies(test_codec_registry_bridge
        pyramid_codec_json_tactical_objects
        pyramid_codec_json_autonomy_backend)
    target_link_libraries(test_codec_registry_bridge PRIVATE
        pcl_core
        pyramid_generated_codecs
        GTest::gtest_main
    )
    target_include_directories(test_codec_registry_bridge PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>
        ${PYRAMID_ROOT_DIR}/tactical_objects/include
        ${PYRAMID_ROOT_DIR}/core/include
    )
    gtest_discover_tests(test_codec_registry_bridge DISCOVERY_MODE PRE_TEST)
endif()

if(PYRAMID_ENABLE_FLATBUFFERS)
    # Showcase for the service-binding facade emitted by the binding
    # generator. Layered as four user files to make the separation explicit:
    #   * tobj_interest_store.{hpp,cpp}    -- typed business logic
    #     (ProvidedHandler subclass).
    #   * tactical_objects_component.hpp   -- hand-written pcl::Component
    #     composing one ProvidedService binding + the store.
    #   * hmi_client_component.hpp         -- hand-written pcl::Component
    #     composing one ConsumedService binding, exposing typed accessors.
    #   * tobj_shared_memory_example.cpp   -- bus bring-up, component
    #     wiring, single-threaded spinOnce loop, demonstration sequence.
    # No PCL types in user code; transport bring-up lives in PCL helpers.
    add_executable(tobj_shared_memory_example
        ${PYRAMID_ROOT_DIR}/examples/cpp/tobj_interest_store.cpp
        ${PYRAMID_ROOT_DIR}/examples/cpp/tobj_shared_memory_example.cpp
        ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
    )
    add_dependencies(tobj_shared_memory_example pyramid_cpp_bindings_codegen)
    target_link_libraries(tobj_shared_memory_example PRIVATE
        pcl_core
        pcl_transport_shared_memory
        pyramid_generated_codecs
        pyramid_flatbuffers_support
    )
    target_include_directories(tobj_shared_memory_example PRIVATE
        ${PCL_ROOT_DIR}/include
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>
    )
    if(NOT WIN32)
        target_link_libraries(tobj_shared_memory_example PRIVATE pthread)
    endif()
    add_test(NAME tobj_shared_memory_example
        COMMAND tobj_shared_memory_example)
    set_tests_properties(tobj_shared_memory_example
        PROPERTIES
            LABELS "example;binding;transport"
            TIMEOUT 15
            RESOURCE_LOCK pyramid_process_e2e)
endif()

add_executable(tobj_socket_server
    ${TOBJ_TEST_DIR}/tobj_socket_server.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
)
add_dependencies(tobj_socket_server pyramid_cpp_bindings_codegen)
target_link_libraries(tobj_socket_server PRIVATE
    tactical_objects tactical_objects_component
    pcl_core pcl_transport_socket
    pyramid_generated_codecs
    pyramid_flatbuffers_support
)
target_include_directories(tobj_socket_server PRIVATE
    ${PCL_ROOT_DIR}/src
    ${PROTO_BINDINGS_DIR}
    ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>
)
if(NOT WIN32)
    target_link_libraries(tobj_socket_server PRIVATE pthread)
endif()

add_executable(standalone_bridge
    ${TOBJ_TEST_DIR}/standalone_bridge.cpp
)
add_dependencies(standalone_bridge pyramid_cpp_bindings_codegen)
target_link_libraries(standalone_bridge PRIVATE
    tactical_objects tactical_objects_component
    pcl_core pcl_transport_socket
    pyramid_generated_codecs
    pyramid_flatbuffers_support
)
target_include_directories(standalone_bridge PRIVATE
    ${PCL_ROOT_DIR}/src
    ${PROTO_BINDINGS_DIR}
)
if(NOT WIN32)
    target_link_libraries(standalone_bridge PRIVATE pthread)
endif()

add_executable(cpp_active_find_e2e EXCLUDE_FROM_ALL
    ${PYRAMID_ROOT_DIR}/tests/tactical_objects/cpp_active_find_e2e.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/tobj_interest_client.cpp
    ${PYRAMID_ROOT_DIR}/examples/cpp/tobj_evidence_provider.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
)
add_dependencies(cpp_active_find_e2e pyramid_cpp_bindings_codegen)
target_link_libraries(cpp_active_find_e2e PRIVATE
    pcl_core pcl_transport_socket
    pyramid_generated_codecs
    pyramid_flatbuffers_support
)
target_include_directories(cpp_active_find_e2e PRIVATE
    ${PCL_ROOT_DIR}/include
    ${PROTO_BINDINGS_DIR}
    ${PYRAMID_ROOT_DIR}/examples/cpp
)
if(NOT WIN32)
    target_link_libraries(cpp_active_find_e2e PRIVATE pthread)
endif()

set(_tobj_cpp_bridge_tests tobj_cpp_bridge_client_e2e)
if(WIN32)
    add_test(NAME tobj_cpp_bridge_client_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.bat
                -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    if(PYRAMID_ENABLE_FLATBUFFERS)
        add_test(NAME tobj_cpp_bridge_client_flatbuffers_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                    -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                    -DSCRIPT_ARGS=--content-type|application/flatbuffers
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        list(APPEND _tobj_cpp_bridge_tests tobj_cpp_bridge_client_flatbuffers_e2e)
    endif()
else()
    add_test(NAME tobj_cpp_bridge_client_e2e
        COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.sh
                --server-bin $<TARGET_FILE:tobj_socket_server>
                --bridge-bin $<TARGET_FILE:standalone_bridge>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>)
    if(PYRAMID_ENABLE_FLATBUFFERS)
        add_test(NAME tobj_cpp_bridge_client_flatbuffers_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin $<TARGET_FILE:tactical_objects_test_client>
                    --content-type application/flatbuffers)
        list(APPEND _tobj_cpp_bridge_tests tobj_cpp_bridge_client_flatbuffers_e2e)
    endif()
endif()
set_tests_properties(
    ${_tobj_cpp_bridge_tests}
    PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 30
        RESOURCE_LOCK pyramid_process_e2e)
unset(_tobj_cpp_bridge_tests)

set(_tobj_cpp_app_tests tobj_cpp_app_client_e2e)
if(WIN32)
    add_test(NAME tobj_cpp_app_client_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    if(PYRAMID_ENABLE_FLATBUFFERS)
        add_test(NAME tobj_cpp_app_client_flatbuffers_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                    -DSCRIPT_ARGS=--content-type|application/flatbuffers
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        list(APPEND _tobj_cpp_app_tests tobj_cpp_app_client_flatbuffers_e2e)
    endif()
    if(PYRAMID_ENABLE_PROTOBUF)
        add_test(NAME tobj_cpp_app_client_protobuf_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                    -DSCRIPT_ARGS=--content-type|application/protobuf
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        list(APPEND _tobj_cpp_app_tests tobj_cpp_app_client_protobuf_e2e)
    endif()
else()
    add_test(NAME tobj_cpp_app_client_e2e
        COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                --app-bin $<TARGET_FILE:tactical_objects_app>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>)
    if(PYRAMID_ENABLE_FLATBUFFERS)
        add_test(NAME tobj_cpp_app_client_flatbuffers_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin $<TARGET_FILE:tactical_objects_test_client>
                    --content-type application/flatbuffers)
        list(APPEND _tobj_cpp_app_tests tobj_cpp_app_client_flatbuffers_e2e)
    endif()
    if(PYRAMID_ENABLE_PROTOBUF)
        add_test(NAME tobj_cpp_app_client_protobuf_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin $<TARGET_FILE:tactical_objects_test_client>
                    --content-type application/protobuf)
        list(APPEND _tobj_cpp_app_tests tobj_cpp_app_client_protobuf_e2e)
    endif()
    if(TARGET pyramid_codec_json_tactical_objects)
        add_dependencies(tactical_objects_app
            pyramid_codec_json_tactical_objects)
        add_dependencies(tactical_objects_test_client
            pyramid_codec_json_tactical_objects)
        add_test(NAME tobj_cpp_app_client_json_plugin_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin $<TARGET_FILE:tactical_objects_test_client>
                    --content-type application/json
                    --codec-plugin $<TARGET_FILE:pyramid_codec_json_tactical_objects>)
        list(APPEND _tobj_cpp_app_tests tobj_cpp_app_client_json_plugin_e2e)
    endif()
    if(PYRAMID_ENABLE_FLATBUFFERS AND TARGET pyramid_codec_flatbuffers_tactical_objects)
        add_dependencies(tactical_objects_app
            pyramid_codec_flatbuffers_tactical_objects)
        add_dependencies(tactical_objects_test_client
            pyramid_codec_flatbuffers_tactical_objects)
        add_test(NAME tobj_cpp_app_client_flatbuffers_plugin_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin $<TARGET_FILE:tactical_objects_test_client>
                    --content-type application/flatbuffers
                    --codec-plugin $<TARGET_FILE:pyramid_codec_flatbuffers_tactical_objects>)
        list(APPEND _tobj_cpp_app_tests tobj_cpp_app_client_flatbuffers_plugin_e2e)
    endif()
endif()
set_tests_properties(
    ${_tobj_cpp_app_tests}
    PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 30
        RESOURCE_LOCK pyramid_process_e2e)
unset(_tobj_cpp_app_tests)

# Config-driven default-plugin load: app + client run with NO --codec-plugin,
# auto-loading the codec from a manifest pointed to by PCL_CODEC_MANIFEST.
if(NOT WIN32 AND TARGET pyramid_codec_json_tactical_objects)
    set(_tobj_codec_manifest
        ${PYRAMID_TEST_GENERATED_DIR}/tobj_codec_manifest.txt)
    file(GENERATE OUTPUT ${_tobj_codec_manifest}
        CONTENT "# auto-loaded codec plugins for the manifest e2e test\n$<TARGET_FILE:pyramid_codec_json_tactical_objects>\n")
    add_test(NAME tobj_cpp_app_client_manifest_e2e
        COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                --app-bin $<TARGET_FILE:tactical_objects_app>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>
                --content-type application/json)
    set_tests_properties(tobj_cpp_app_client_manifest_e2e PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 30
        ENVIRONMENT "PCL_CODEC_MANIFEST=${_tobj_codec_manifest}"
        RESOURCE_LOCK pyramid_process_e2e)
endif()

if(WIN32 AND TARGET pyramid_grpc_transport)
    add_executable(tobj_grpc_server
        tobj_grpc_server.cpp
    )
    target_link_libraries(tobj_grpc_server PRIVATE
        pyramid_grpc_transport
        pyramid_protobuf_support
    )
    target_include_directories(tobj_grpc_server PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp
        ${PYRAMID_PROTOBUF_GEN_DIR}
    )
endif()

find_program(GPRBUILD gprbuild)
set(PYRAMID_PROTO_ROOT ${PYRAMID_ROOT_DIR}/proto)
if(GPRBUILD AND EXISTS "${PYRAMID_PROTO_ROOT}")
    set(ADA_RUN_IF_MISSING ${UNMANNED_WORKSPACE_ROOT}/cmake/run_if_missing.cmake)
    set(ADA_TEST_SRC_DIR ${PYRAMID_ROOT_DIR}/tests/ada)
    find_program(ADA_PYTHON NAMES python3 python)
    if(NOT ADA_PYTHON)
        message(FATAL_ERROR "Python is required to generate Ada bindings for tests")
    endif()
    set(BINDINGS_GEN_SCRIPT ${PYRAMID_ROOT_DIR}/pim/generate_bindings.py)
    set(ADA_GEN_OUT ${PYRAMID_ROOT_DIR}/bindings/ada/generated)
    set(ADA_GNATCOLL_CORE_DIR ${PYRAMID_ROOT_DIR}/core/external/gnatcoll-core/core)
    set(ADA_GNATCOLL_MINIMAL_DIR ${PYRAMID_ROOT_DIR}/core/external/gnatcoll-core/minimal)
    set(ADA_GPR_PROJECT_DIRS
        ${ADA_GNATCOLL_CORE_DIR}
        ${ADA_GNATCOLL_MINIMAL_DIR})
    set(ADA_GPRBUILD_PROJECT_ARGS "")
    set(_ada_gprbuild_project_args "")
    foreach(_ada_gpr_project_dir IN LISTS ADA_GPR_PROJECT_DIRS)
        list(APPEND ADA_GPRBUILD_PROJECT_ARGS -aP ${_ada_gpr_project_dir})
        string(APPEND _ada_gprbuild_project_args ";-aP;${_ada_gpr_project_dir}")
    endforeach()
    if(WIN32)
        set(ADA_PROJECT_OS_VALUE Windows_NT)
        set(ADA_GNATCOLL_OS_VALUE windows)
    elseif(APPLE)
        set(ADA_PROJECT_OS_VALUE darwin)
        set(ADA_GNATCOLL_OS_VALUE osx)
    else()
        set(ADA_PROJECT_OS_VALUE linux)
        set(ADA_GNATCOLL_OS_VALUE unix)
    endif()
    set(ADA_GPRBUILD_PLATFORM_ARGS
        -XOS=${ADA_PROJECT_OS_VALUE}
        -XGNATCOLL_OS=${ADA_GNATCOLL_OS_VALUE})
    set(_ada_gprbuild_platform_args
        ";-XOS=${ADA_PROJECT_OS_VALUE};-XGNATCOLL_OS=${ADA_GNATCOLL_OS_VALUE}")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++.so
        OUTPUT_VARIABLE ADA_CXX_STDLIB_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(ADA_CXX_STDLIB_PATH AND IS_ABSOLUTE "${ADA_CXX_STDLIB_PATH}")
        get_filename_component(ADA_CXX_LIB_DIR
            "${ADA_CXX_STDLIB_PATH}" DIRECTORY)
    else()
        set(ADA_CXX_LIB_DIR "")
    endif()
    set(ADA_PCL_INCLUDE_DIR "${PCL_ROOT_DIR}/include" CACHE PATH
        "Include directory for Ada gprbuild PCL headers")
    set(ADA_PCL_LIB_NAME "pcl_core" CACHE STRING
        "Base library name for Ada gprbuild core PCL static library")
    set(ADA_PCL_SOCKET_LIB_NAME "pcl_transport_socket" CACHE STRING
        "Base library name for Ada gprbuild PCL socket transport static library")
    if(WIN32)
        set(ADA_PCL_LIB_DIR "${CMAKE_BINARY_DIR}/ada_gnat_pcl" CACHE PATH
            "Directory containing GNAT-compatible libpcl_core.a and libpcl_transport_socket.a")
        set(ADA_PCL_BUILD_SCRIPT "${PCL_ROOT_DIR}/scripts/build_gnat_pcl_static_libs.bat")
    else()
        set(ADA_PCL_LIB_DIR "${CMAKE_BINARY_DIR}/subprojects/PCL/src" CACHE PATH
            "Directory containing libpcl_core/libpcl_transport_socket for Ada gprbuild")
    endif()

    # Aggregate target: explicitly excluded from ALL so routine builds stay fast.
    # Ada binaries are expensive and GNAT/gprbuild may not be installed.
    # Build with: cmake --build --preset release --target pyramid_ada_all
    add_custom_target(pyramid_ada_all
        COMMENT "Build all Ada test binaries and bridges")

    set(ADA_PCL_SHMEM_LIB_NAME "pcl_transport_shared_memory" CACHE STRING
        "Base library name for Ada gprbuild PCL shared-memory transport static library")
    set(ADA_PCL_UDP_LIB_NAME "pcl_transport_udp" CACHE STRING
        "Base library name for Ada gprbuild PCL UDP transport static library")

    set(ADA_PCL_ARCHIVES
        ${ADA_PCL_LIB_DIR}/lib${ADA_PCL_LIB_NAME}.a
        ${ADA_PCL_LIB_DIR}/lib${ADA_PCL_SOCKET_LIB_NAME}.a
        ${ADA_PCL_LIB_DIR}/lib${ADA_PCL_UDP_LIB_NAME}.a
        ${ADA_PCL_LIB_DIR}/lib${ADA_PCL_SHMEM_LIB_NAME}.a)
    if(WIN32)
        add_custom_command(
            OUTPUT ${ADA_PCL_ARCHIVES}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PCL_LIB_DIR}
            COMMAND cmd /c "${ADA_PCL_BUILD_SCRIPT}" "${ADA_PCL_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible PCL static archives for Ada builds"
            VERBATIM)
        add_custom_target(ada_pcl_gnat_archives
            DEPENDS ${ADA_PCL_ARCHIVES})
    else()
        add_custom_target(ada_pcl_gnat_archives)
        add_dependencies(ada_pcl_gnat_archives
            pcl_core
            pcl_transport_socket
            pcl_transport_udp
            pcl_transport_shared_memory)
    endif()

    set(ADA_EXAMPLE_DIR ${PYRAMID_ROOT_DIR}/examples/ada)
    set(ADA_PCL_SENSOR_DEMO_SOURCES
        ${ADA_EXAMPLE_DIR}/pcl_sensor_demo.adb
        ${ADA_EXAMPLE_DIR}/pcl_sensor_demo.gpr)
    set(ADA_PCL_COMPONENT_TESTS_SOURCES
        ${ADA_TEST_SRC_DIR}/pcl_component_tests.adb
        ${ADA_TEST_SRC_DIR}/pcl_component_tests.gpr)
    set(ADA_PCL_PLUGIN_LOADER_TEST_SOURCES
        ${ADA_TEST_SRC_DIR}/test_pcl_plugin_loader.adb
        ${ADA_TEST_SRC_DIR}/test_pcl_plugin_loader.gpr
        ${PCL_ROOT_DIR}/bindings/ada/pcl_plugins.ads)
    set(ADA_PYRAMID_UUID_TEST_SOURCES
        ${ADA_TEST_SRC_DIR}/test_pyramid_uuids.adb
        ${ADA_TEST_SRC_DIR}/test_pyramid_uuids.gpr
        ${ADA_EXAMPLE_DIR}/pyramid-uuids.adb
        ${ADA_EXAMPLE_DIR}/pyramid-uuids.ads)
    set(ADA_PCL_SENSOR_DEMO_EXE
        ${ADA_EXAMPLE_DIR}/bin/pcl_sensor_demo${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_PCL_SENSOR_DEMO_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_EXAMPLE_DIR}/bin
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_PCL_SENSOR_DEMO_EXE}
                -DWORKING_DIRECTORY=${ADA_EXAMPLE_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_EXAMPLE_DIR}/pcl_sensor_demo.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_EXAMPLE_DIR}
        DEPENDS ${ADA_PCL_SENSOR_DEMO_SOURCES}
        COMMENT "Building Ada PCL sensor demo"
        VERBATIM)
    add_custom_target(ada_pcl_sensor_demo_build
        DEPENDS ${ADA_PCL_SENSOR_DEMO_EXE})
    add_dependencies(ada_pcl_sensor_demo_build ada_pcl_gnat_archives)

    set(ADA_PCL_COMPONENT_TESTS_EXE
        ${ADA_TEST_SRC_DIR}/bin/pcl_component_tests${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_PCL_COMPONENT_TESTS_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_PCL_COMPONENT_TESTS_EXE}
                -DWORKING_DIRECTORY=${ADA_TEST_SRC_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_TEST_SRC_DIR}/pcl_component_tests.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
        DEPENDS ${ADA_PCL_COMPONENT_TESTS_SOURCES}
        COMMENT "Building Ada PCL component tests"
        VERBATIM)
    add_custom_target(ada_pcl_component_tests_build
        DEPENDS ${ADA_PCL_COMPONENT_TESTS_EXE})
    add_dependencies(ada_pcl_component_tests_build ada_pcl_gnat_archives)

    set(ADA_PCL_PLUGIN_LOADER_TEST_EXE
        ${ADA_TEST_SRC_DIR}/bin/test_pcl_plugin_loader${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_PCL_PLUGIN_LOADER_TEST_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_PCL_PLUGIN_LOADER_TEST_EXE}
                -DWORKING_DIRECTORY=${ADA_TEST_SRC_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_TEST_SRC_DIR}/test_pcl_plugin_loader.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
        DEPENDS ${ADA_PCL_PLUGIN_LOADER_TEST_SOURCES}
        COMMENT "Building Ada PCL plugin-loader ABI test"
        VERBATIM)
    add_custom_target(ada_pcl_plugin_loader_tests_build
        DEPENDS ${ADA_PCL_PLUGIN_LOADER_TEST_EXE})
    add_dependencies(ada_pcl_plugin_loader_tests_build
        ada_pcl_gnat_archives
        pcl_codec_stub_plugin)

    set(ADA_PYRAMID_UUID_TEST_EXE
        ${ADA_TEST_SRC_DIR}/bin/test_pyramid_uuids${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_PYRAMID_UUID_TEST_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_PYRAMID_UUID_TEST_EXE}
                -DWORKING_DIRECTORY=${ADA_TEST_SRC_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_TEST_SRC_DIR}/test_pyramid_uuids.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
        DEPENDS ${ADA_PYRAMID_UUID_TEST_SOURCES}
        COMMENT "Building Ada Pyramid UUID test"
        VERBATIM)
    add_custom_target(ada_pyramid_uuid_test_build
        DEPENDS ${ADA_PYRAMID_UUID_TEST_EXE})

    add_dependencies(pyramid_ada_all
        ada_pcl_gnat_archives
        ada_pcl_sensor_demo_build
        ada_pcl_component_tests_build
        ada_pcl_plugin_loader_tests_build
        ada_pyramid_uuid_test_build)

    set(_pyramid_ada_tests)

    add_test(NAME ada_plugin_loader_abi
        COMMAND ${ADA_TEST_SRC_DIR}/bin/test_pcl_plugin_loader
                $<TARGET_FILE:pcl_codec_stub_plugin>)
    set_tests_properties(ada_plugin_loader_abi PROPERTIES
        LABELS "ada;cross_language;plugin;codec"
        TIMEOUT 20)
    list(APPEND _pyramid_ada_tests ada_plugin_loader_abi)

    if(PYRAMID_ENABLE_FLATBUFFERS)
        set(ADA_PYRAMID_GEN_LIB_NAME "pyramid_generated_flatbuffers_codec" CACHE STRING
            "Base library name for Ada gprbuild generated FlatBuffers codec archive")
        if(WIN32)
            set(ADA_PYRAMID_GEN_LIB_DIR "${CMAKE_BINARY_DIR}/ada_gnat_pyramid" CACHE PATH
                "Directory containing GNAT-compatible generated FlatBuffers codec archive")
            set(ADA_PYRAMID_GEN_BUILD_SCRIPT "${PYRAMID_ROOT_DIR}/scripts/build_gnat_generated_flatbuffers_libs.bat")
        else()
            set(ADA_PYRAMID_GEN_LIB_DIR "${CMAKE_BINARY_DIR}/ada_gnat_pyramid" CACHE PATH
                "Directory containing GNAT-compatible generated FlatBuffers codec archive")
            set(ADA_PYRAMID_GEN_BUILD_SCRIPT "${PYRAMID_ROOT_DIR}/scripts/build_gnat_generated_flatbuffers_libs.sh")
        endif()
        set(_ada_fb_args
            -XPYRAMID_GEN_LIB_DIR=${ADA_PYRAMID_GEN_LIB_DIR}
            -XPYRAMID_GEN_LIB_NAME=${ADA_PYRAMID_GEN_LIB_NAME}
            -XPYRAMID_CODECS_LIB_DIR=${CMAKE_BINARY_DIR}/subprojects/PYRAMID
            -XPYRAMID_CODECS_LIB_NAME=pyramid_generated_marshal
        )
        if(ADA_CXX_LIB_DIR)
            list(APPEND _ada_fb_args -XCXX_LIB_DIR=${ADA_CXX_LIB_DIR})
        endif()
        set(_ada_fb_mkdir COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PYRAMID_GEN_LIB_DIR})
        file(GLOB ADA_PYRAMID_GEN_CODEC_INPUTS CONFIGURE_DEPENDS
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/pyramid_data_model_*_codec.cpp"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/pyramid_data_model_*_codec.hpp"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/flatbuffers/cpp/*_flatbuffers_codec.cpp"
            "${PYRAMID_ROOT_DIR}/bindings/cpp/generated/flatbuffers/cpp/*_flatbuffers_codec.hpp")

    set(ADA_PYRAMID_GEN_ARCHIVE
        ${ADA_PYRAMID_GEN_LIB_DIR}/lib${ADA_PYRAMID_GEN_LIB_NAME}.a)
    if(WIN32)
        add_custom_command(
            OUTPUT ${ADA_PYRAMID_GEN_ARCHIVE}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PYRAMID_GEN_LIB_DIR}
            COMMAND cmd /c ${ADA_PYRAMID_GEN_BUILD_SCRIPT} ${ADA_PYRAMID_GEN_LIB_DIR} --build-dir ${CMAKE_BINARY_DIR} --config $<CONFIG>
            DEPENDS
                ${ADA_PYRAMID_GEN_BUILD_SCRIPT}
                ${ADA_PYRAMID_GEN_CODEC_INPUTS}
                pyramid_flatbuffers_codegen
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Ada builds"
            VERBATIM)
    else()
        add_custom_command(
            OUTPUT ${ADA_PYRAMID_GEN_ARCHIVE}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PYRAMID_GEN_LIB_DIR}
            COMMAND bash ${ADA_PYRAMID_GEN_BUILD_SCRIPT} ${ADA_PYRAMID_GEN_LIB_DIR} --build-dir ${CMAKE_BINARY_DIR}
            DEPENDS
                ${ADA_PYRAMID_GEN_BUILD_SCRIPT}
                ${ADA_PYRAMID_GEN_CODEC_INPUTS}
                pyramid_flatbuffers_codegen
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Ada builds"
            VERBATIM)
    endif()
    add_custom_target(ada_pyramid_generated_flatbuffers_archive
        DEPENDS ${ADA_PYRAMID_GEN_ARCHIVE})

    set(_ada_bindings_backends json)
    if(PYRAMID_ENABLE_FLATBUFFERS)
        list(APPEND _ada_bindings_backends flatbuffers)
    endif()
    if(PYRAMID_ENABLE_PROTOBUF)
        list(APPEND _ada_bindings_backends protobuf)
    endif()
    if(PYRAMID_ENABLE_GRPC)
        list(APPEND _ada_bindings_backends grpc)
    endif()
    if(PYRAMID_ENABLE_ROS2)
        list(APPEND _ada_bindings_backends ros2)
    endif()
    list(JOIN _ada_bindings_backends "," _ada_bindings_backends_arg)

    set(ADA_GENERATED_BINDINGS_EXE
        ${ADA_TEST_SRC_DIR}/bin/test_generated_bindings${CMAKE_EXECUTABLE_SUFFIX})
    set(_ada_generated_bindings_commands
        COMMAND ${ADA_PYTHON}
                ${BINDINGS_GEN_SCRIPT}
                ${PYRAMID_PROTO_ROOT}
                ${ADA_GEN_OUT}
                --languages ada
                --backends ${_ada_bindings_backends_arg}
    )
    if(PYRAMID_ENABLE_PROTOBUF)
        list(APPEND _ada_generated_bindings_commands
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${ADA_GEN_OUT}/protobuf/ada/pyramid_data_model_base-protobuf_codec.ads
                    ${ADA_GEN_OUT}/protobuf/ada/pyramid-data_model-base-protobuf_codec.ads
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${ADA_GEN_OUT}/protobuf/ada/pyramid_data_model_common-protobuf_codec.ads
                    ${ADA_GEN_OUT}/protobuf/ada/pyramid-data_model-common-protobuf_codec.ads
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${ADA_GEN_OUT}/protobuf/ada/pyramid_data_model_tactical-protobuf_codec.ads
                    ${ADA_GEN_OUT}/protobuf/ada/pyramid-data_model-tactical-protobuf_codec.ads
        )
    endif()
    list(APPEND _ada_generated_bindings_commands
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PCL_LIB_DIR}
        ${_ada_fb_mkdir}
        COMMAND ${CMAKE_COMMAND} -E env
                UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                ${GPRBUILD}
                ${ADA_GPRBUILD_PROJECT_ARGS}
                ${ADA_GPRBUILD_PLATFORM_ARGS}
                -p
                -P ${ADA_TEST_SRC_DIR}/test_generated_bindings.gpr
                -XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                -XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR}
                -XPCL_LIB_DIR=${ADA_PCL_LIB_DIR}
                -XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}
                -XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME}
                -XPYRAMID_PROTOBUF_ADA_DIR=${ADA_GEN_OUT}/protobuf/ada
                -XPYRAMID_GRPC_ADA_DIR=${ADA_GEN_OUT}/grpc/ada
                ${_ada_fb_args}
    )
    add_custom_command(
        OUTPUT ${ADA_GENERATED_BINDINGS_EXE}
        ${_ada_generated_bindings_commands}
        DEPENDS
            ${PYRAMID_BINDING_GENERATOR_INPUTS}
            ${PYRAMID_PROTO_INPUTS}
            ${ADA_TEST_SRC_DIR}/test_generated_bindings.adb
            ${ADA_TEST_SRC_DIR}/test_generated_bindings.gpr
        WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
        COMMENT "Building Ada generated-bindings test"
        VERBATIM
    )
    add_custom_target(ada_test_generated_bindings_build
        DEPENDS ${ADA_GENERATED_BINDINGS_EXE})
    add_dependencies(ada_test_generated_bindings_build
        ada_pcl_gnat_archives
        ada_pyramid_generated_flatbuffers_archive
        pyramid_generated_codecs)

    add_test(NAME ada_generated_bindings_roundtrip
        COMMAND ${ADA_TEST_SRC_DIR}/bin/test_generated_bindings)
    set_tests_properties(ada_generated_bindings_roundtrip PROPERTIES
        LABELS "ada;generated;codec"
        TIMEOUT 20
        ENVIRONMENT "PYRAMID_CODEC_PLUGINS=${_pyramid_codec_plugin_env}")
    list(APPEND _pyramid_ada_tests ada_generated_bindings_roundtrip)


    if(TARGET pyramid_codec_json_tactical_objects)
        set(ADA_CPP_CODEC_ROUNDTRIP_EXE
            ${ADA_TEST_SRC_DIR}/bin/test_ada_cpp_codec_roundtrip${CMAKE_EXECUTABLE_SUFFIX})
        add_custom_command(
            OUTPUT ${ADA_CPP_CODEC_ROUNDTRIP_EXE}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
            COMMAND ${CMAKE_COMMAND} -E env
                    UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                    ${GPRBUILD}
                    ${ADA_GPRBUILD_PROJECT_ARGS}
                    ${ADA_GPRBUILD_PLATFORM_ARGS}
                    -p
                    -P ${ADA_TEST_SRC_DIR}/test_ada_cpp_codec_roundtrip.gpr
                    -XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                    -XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR}
                    -XPCL_LIB_DIR=${ADA_PCL_LIB_DIR}
                    -XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}
                    -XPYRAMID_CODECS_LIB_DIR=${CMAKE_BINARY_DIR}/subprojects/PYRAMID
                    -XPYRAMID_CODECS_LIB_NAME=pyramid_generated_marshal
                    -XCXX_LIB_DIR=${ADA_CXX_LIB_DIR}
            WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
            DEPENDS
                ${ADA_TEST_SRC_DIR}/test_ada_cpp_codec_roundtrip.adb
                ${ADA_TEST_SRC_DIR}/test_ada_cpp_codec_roundtrip.gpr
                ${PCL_ROOT_DIR}/bindings/ada/pcl_plugins.ads
                ${PCL_ROOT_DIR}/bindings/ada/pcl_bindings.ads
                ${PYRAMID_BINDING_GENERATOR_INPUTS}
                ${PYRAMID_PROTO_INPUTS}
                pyramid_generated_codecs
                pyramid_codec_json_tactical_objects
            COMMENT "Building Ada/C++ shared codec-plugin round-trip test"
            VERBATIM)
        add_custom_target(ada_cpp_codec_roundtrip_build
            DEPENDS ${ADA_CPP_CODEC_ROUNDTRIP_EXE})
        add_dependencies(ada_cpp_codec_roundtrip_build
            ada_test_generated_bindings_build
            ada_pcl_gnat_archives
            pyramid_generated_codecs
            pyramid_codec_json_tactical_objects)

        add_dependencies(pyramid_ada_all
            ada_cpp_codec_roundtrip_build)

        add_test(NAME ada_cpp_codec_roundtrip
            COMMAND ${ADA_CPP_CODEC_ROUNDTRIP_EXE}
                    $<TARGET_FILE:pyramid_codec_json_tactical_objects>)
        set_tests_properties(ada_cpp_codec_roundtrip PROPERTIES
            LABELS "ada;cross_language;plugin;codec"
            TIMEOUT 20)
        list(APPEND _pyramid_ada_tests ada_cpp_codec_roundtrip)
    endif()

    # Plugin-only Ada clients load BOTH the socket transport and the codec from
    # .so plugins at run time. Thread the transport path through
    # PCL_TRANSPORT_PLUGIN and the codec paths through PYRAMID_CODEC_PLUGINS so
    # the (now fail-closed) facade has a registered codec without a static
    # fallback (uniform with the C++ client).
    set(_ada_plugin_env "")
    if(TARGET pcl_transport_socket_plugin)
        list(APPEND _ada_plugin_env
            "PCL_TRANSPORT_PLUGIN=$<TARGET_FILE:pcl_transport_socket_plugin>")
    endif()
    if(TARGET pyramid_codec_json_tactical_objects AND
       TARGET pyramid_codec_flatbuffers_tactical_objects)
        list(APPEND _ada_plugin_env
            "PYRAMID_CODEC_PLUGINS=$<TARGET_FILE:pyramid_codec_json_tactical_objects>:$<TARGET_FILE:pyramid_codec_flatbuffers_tactical_objects>")
    elseif(TARGET pyramid_codec_json_tactical_objects)
        list(APPEND _ada_plugin_env
            "PYRAMID_CODEC_PLUGINS=$<TARGET_FILE:pyramid_codec_json_tactical_objects>")
    endif()

    if(WIN32)
        add_test(NAME tobj_ada_socket_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_socket_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_tobj_client
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_socket_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_socket_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_tobj_client)
    endif()
    set_tests_properties(tobj_ada_socket_e2e PROPERTIES
        LABELS "cross_language;integration"
        TIMEOUT 60
        ENVIRONMENT "${_ada_plugin_env}"
        RESOURCE_LOCK pyramid_process_e2e)
    list(APPEND _pyramid_ada_tests tobj_ada_socket_e2e)

    if(NOT WIN32 AND TARGET pyramid_codec_json_tactical_objects)
        add_test(NAME tobj_ada_socket_json_plugin_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_socket_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_tobj_client
                    --content-type application/json
                    # The Ada client marshals native Ada records (not the C-ABI
                    # struct the C++ plugin expects), so it loads the Ada
                    # raw-record codec plugin here.
                    --codec-plugin $<TARGET_FILE:pyramid_codec_json_tactical_objects>)
        set_tests_properties(tobj_ada_socket_json_plugin_e2e PROPERTIES
            LABELS "cross_language;integration;codec"
            TIMEOUT 60
            ENVIRONMENT "${_ada_plugin_env}"
            RESOURCE_LOCK pyramid_process_e2e)
        list(APPEND _pyramid_ada_tests tobj_ada_socket_json_plugin_e2e)
    endif()

    if(WIN32)
        add_test(NAME tobj_ada_active_find_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e)
    endif()
    set_tests_properties(tobj_ada_active_find_e2e PROPERTIES
        LABELS "cross_language;integration"
        TIMEOUT 40
        ENVIRONMENT "${_ada_plugin_env}"
        RESOURCE_LOCK pyramid_process_e2e)
    list(APPEND _pyramid_ada_tests tobj_ada_active_find_e2e)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_flatbuffers_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    -DSCRIPT_ARGS=--content-type|application/flatbuffers
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_flatbuffers_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    --content-type application/flatbuffers)
    endif()
    set_tests_properties(tobj_ada_active_find_flatbuffers_e2e PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 40
        ENVIRONMENT "${_ada_plugin_env}"
        RESOURCE_LOCK pyramid_process_e2e)
    list(APPEND _pyramid_ada_tests tobj_ada_active_find_flatbuffers_e2e)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_app_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_app_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e)
    endif()
    set_tests_properties(tobj_ada_active_find_app_e2e PROPERTIES
        LABELS "cross_language;integration;real_app"
        TIMEOUT 40
        ENVIRONMENT "${_ada_plugin_env}"
        RESOURCE_LOCK pyramid_process_e2e)
    list(APPEND _pyramid_ada_tests tobj_ada_active_find_app_e2e)

    if(NOT WIN32)
        add_test(NAME tobj_ada_active_find_app_json_plugin_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    --content-type application/json
                    --codec-plugin $<TARGET_FILE:pyramid_codec_json_tactical_objects>)
        set_tests_properties(tobj_ada_active_find_app_json_plugin_e2e PROPERTIES
            LABELS "cross_language;integration;codec;real_app"
            TIMEOUT 40
            ENVIRONMENT "${_ada_plugin_env}"
            RESOURCE_LOCK pyramid_process_e2e)
        list(APPEND _pyramid_ada_tests tobj_ada_active_find_app_json_plugin_e2e)
    endif()

    if(WIN32)
        add_test(NAME tobj_ada_active_find_app_flatbuffers_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    -DSCRIPT_ARGS=--content-type|application/flatbuffers
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_app_flatbuffers_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    --content-type application/flatbuffers)
    endif()
    set_tests_properties(tobj_ada_active_find_app_flatbuffers_e2e PROPERTIES
        LABELS "cross_language;integration;codec;real_app"
        TIMEOUT 40
        ENVIRONMENT "${_ada_plugin_env}"
        RESOURCE_LOCK pyramid_process_e2e)
    list(APPEND _pyramid_ada_tests tobj_ada_active_find_app_flatbuffers_e2e)

    # -- Pyramid Bridge Ada build + E2E test -----------------------------------
    set(ADA_BRIDGE_SRC_DIR ${PYRAMID_ROOT_DIR}/pyramid_bridge/ada)
    file(GLOB ADA_BRIDGE_SOURCES CONFIGURE_DEPENDS
        "${ADA_BRIDGE_SRC_DIR}/*.adb"
        "${ADA_BRIDGE_SRC_DIR}/*.ads"
        "${ADA_BRIDGE_SRC_DIR}/*.gpr")

    set(ADA_PYRAMID_BRIDGE_EXE
        ${ADA_BRIDGE_SRC_DIR}/bin/pyramid_bridge_main${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_PYRAMID_BRIDGE_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_BRIDGE_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PCL_LIB_DIR}
        ${_ada_fb_mkdir}
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_PYRAMID_BRIDGE_EXE}
                -DWORKING_DIRECTORY=${ADA_BRIDGE_SRC_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_BRIDGE_SRC_DIR}/pyramid_bridge.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME};-XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME};-XPCL_UDP_LIB_NAME=${ADA_PCL_UDP_LIB_NAME};-XPCL_SHMEM_LIB_NAME=${ADA_PCL_SHMEM_LIB_NAME};${_ada_fb_args}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_BRIDGE_SRC_DIR}
        DEPENDS
            ${ADA_BRIDGE_SOURCES}
            ${PYRAMID_BINDING_GENERATOR_INPUTS}
            ${PYRAMID_PROTO_INPUTS}
        COMMENT "Building Ada Pyramid Bridge"
        VERBATIM
    )
    add_custom_target(pyramid_bridge_ada_build
        DEPENDS ${ADA_PYRAMID_BRIDGE_EXE})
    add_dependencies(pyramid_bridge_ada_build
        ada_test_generated_bindings_build
        ada_pcl_gnat_archives
        ada_pyramid_generated_flatbuffers_archive)

    if(WIN32)
        add_test(NAME pyramid_bridge_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_pyramid_bridge_e2e.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DSTUB_BIN=$<TARGET_FILE:ame_backend_stub>
                    -DEVIDENCE_BIN=$<TARGET_FILE:pyramid_bridge_evidence_client>
                    -DBRIDGE_BIN=${PYRAMID_ROOT_DIR}/pyramid_bridge/ada/bin/pyramid_bridge_main
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME pyramid_bridge_e2e
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_pyramid_bridge_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --stub-bin $<TARGET_FILE:ame_backend_stub>
                    --evidence-bin $<TARGET_FILE:pyramid_bridge_evidence_client>
                    --bridge-bin ${PYRAMID_ROOT_DIR}/pyramid_bridge/ada/bin/pyramid_bridge_main)
    endif()
    set_tests_properties(pyramid_bridge_e2e PROPERTIES
        LABELS "cross_language;integration;shared_memory;real_app"
        TIMEOUT 60
        RESOURCE_LOCK pyramid_process_e2e)
    list(APPEND _pyramid_ada_tests pyramid_bridge_e2e)

    # Ada example client — built by cmake --build --target pyramid_ada_all
    set(ADA_EXAMPLE_DIR ${PYRAMID_ROOT_DIR}/examples/ada)
    file(GLOB ADA_EXAMPLE_SOURCES CONFIGURE_DEPENDS
        "${ADA_EXAMPLE_DIR}/*.adb"
        "${ADA_EXAMPLE_DIR}/*.ads"
        "${ADA_EXAMPLE_DIR}/*.gpr")
    set(ADA_TOBJ_CLIENT_EXE
        ${ADA_EXAMPLE_DIR}/bin/ada_tobj_client${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_TOBJ_CLIENT_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_EXAMPLE_DIR}/bin
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_TOBJ_CLIENT_EXE}
                -DWORKING_DIRECTORY=${ADA_EXAMPLE_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_EXAMPLE_DIR}/ada_tobj_client.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME};-XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME};${_ada_fb_args}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_EXAMPLE_DIR}
        DEPENDS
            ${ADA_EXAMPLE_SOURCES}
            ${PYRAMID_BINDING_GENERATOR_INPUTS}
            ${PYRAMID_PROTO_INPUTS}
        COMMENT "Building Ada tactical objects client"
        VERBATIM
    )
    add_custom_target(ada_tobj_client_build
        DEPENDS ${ADA_TOBJ_CLIENT_EXE})
    add_dependencies(ada_tobj_client_build
        ada_test_generated_bindings_build
        ada_pcl_gnat_archives
        ada_pyramid_generated_flatbuffers_archive
        pyramid_generated_codecs)

    # Ada active-find E2E client — built by cmake --build --target pyramid_ada_all
    set(ADA_ACTIVE_FIND_E2E_EXE
        ${ADA_TEST_SRC_DIR}/bin/ada_active_find_e2e${CMAKE_EXECUTABLE_SUFFIX})
    add_custom_command(
        OUTPUT ${ADA_ACTIVE_FIND_E2E_EXE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT=${ADA_ACTIVE_FIND_E2E_EXE}
                -DWORKING_DIRECTORY=${ADA_TEST_SRC_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_TEST_SRC_DIR}/ada_active_find_e2e.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME};-XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME};${_ada_fb_args}"
                -P ${ADA_RUN_IF_MISSING}
        WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
        DEPENDS
            ${ADA_TEST_SRC_DIR}/ada_active_find_e2e.adb
            ${ADA_TEST_SRC_DIR}/ada_active_find_e2e.gpr
            ${ADA_EXAMPLE_SOURCES}
            ${PYRAMID_BINDING_GENERATOR_INPUTS}
            ${PYRAMID_PROTO_INPUTS}
        COMMENT "Building Ada active-find E2E test client"
        VERBATIM
    )
    add_custom_target(ada_active_find_e2e_build
        DEPENDS ${ADA_ACTIVE_FIND_E2E_EXE})
    add_dependencies(ada_active_find_e2e_build
        ada_test_generated_bindings_build
        ada_pcl_gnat_archives
        ada_pyramid_generated_flatbuffers_archive
        pyramid_generated_codecs)

    add_dependencies(pyramid_ada_all
        ada_test_generated_bindings_build
        pyramid_bridge_ada_build
        ada_tobj_client_build
        ada_active_find_e2e_build)

    endif() # PYRAMID_ENABLE_FLATBUFFERS

    if(WIN32 AND TARGET pyramid_grpc_transport)
        set(ADA_GRPC_CPP_INTEROP_EXE
            ${ADA_TEST_SRC_DIR}/bin/ada_grpc_cpp_interop_e2e${CMAKE_EXECUTABLE_SUFFIX})
        add_custom_command(
            OUTPUT ${ADA_GRPC_CPP_INTEROP_EXE}
            COMMAND ${CMAKE_COMMAND}
                    -DOUTPUT=${ADA_GRPC_CPP_INTEROP_EXE}
                    -DWORKING_DIRECTORY=${ADA_TEST_SRC_DIR}
                "-DRUN_COMMAND=${CMAKE_COMMAND};-E;env;UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};${GPRBUILD}${_ada_gprbuild_project_args}${_ada_gprbuild_platform_args};-p;-P;${ADA_TEST_SRC_DIR}/ada_grpc_cpp_interop_e2e.gpr;-q;-XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT};-XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR};-XPCL_LIB_DIR=${ADA_PCL_LIB_DIR};-XPCL_LIB_NAME=${ADA_PCL_LIB_NAME};-XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME};-XPYRAMID_GEN_LIB_DIR=${ADA_PYRAMID_GEN_LIB_DIR};-XPYRAMID_GEN_LIB_NAME=${ADA_PYRAMID_GEN_LIB_NAME};-XPYRAMID_CODECS_LIB_DIR=${CMAKE_BINARY_DIR}/subprojects/PYRAMID;-XPYRAMID_CODECS_LIB_NAME=pyramid_generated_marshal;-XPYRAMID_PROTOBUF_ADA_DIR=${ADA_GEN_OUT}/protobuf/ada;-XPYRAMID_GRPC_ADA_DIR=${ADA_GEN_OUT}/grpc/ada"
                    -P ${ADA_RUN_IF_MISSING}
            DEPENDS
                ${PYRAMID_BINDING_GENERATOR_INPUTS}
                ${PYRAMID_PROTO_INPUTS}
                ${ADA_TEST_SRC_DIR}/ada_grpc_cpp_interop_e2e.adb
                ${ADA_TEST_SRC_DIR}/ada_grpc_cpp_interop_e2e.gpr
            WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
            COMMENT "Building Ada gRPC C++ interop test"
            VERBATIM
        )
        add_custom_target(ada_grpc_cpp_interop_build
            DEPENDS ${ADA_GRPC_CPP_INTEROP_EXE})
        add_dependencies(ada_grpc_cpp_interop_build
            pyramid_grpc_c_shim
            ada_test_generated_bindings_build
            ada_pcl_gnat_archives
            ada_pyramid_generated_flatbuffers_archive)
        add_dependencies(pyramid_ada_all ada_grpc_cpp_interop_build)

        add_test(NAME tobj_ada_grpc_cpp_interop_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_grpc_cpp_interop_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_grpc_server>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_grpc_cpp_interop_e2e
                    -DSCRIPT_ARGS=--dll-bin|$<TARGET_FILE:pyramid_grpc_c_shim>|--address|auto
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        set_tests_properties(tobj_ada_grpc_cpp_interop_e2e PROPERTIES
            LABELS "cross_language;integration;grpc"
            TIMEOUT 30
            RESOURCE_LOCK pyramid_process_e2e)
        list(APPEND _pyramid_ada_tests tobj_ada_grpc_cpp_interop_e2e)

        add_test(NAME tobj_master_conformance_e2e
                    COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_tobj_master_conformance.bat
                    -DSCRIPT_ARGS=--socket-server-bin|$<TARGET_FILE:tobj_socket_server>|--bridge-bin|$<TARGET_FILE:standalone_bridge>|--socket-client-bin|${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e|--grpc-server-bin|$<TARGET_FILE:tobj_grpc_server>|--grpc-client-bin|${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_grpc_cpp_interop_e2e|--grpc-dll-bin|$<TARGET_FILE:pyramid_grpc_c_shim>
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        set_tests_properties(tobj_master_conformance_e2e PROPERTIES
            LABELS "cross_language;integration;conformance;codec;grpc"
            TIMEOUT 120
            RESOURCE_LOCK pyramid_process_e2e)
        list(APPEND _pyramid_ada_tests tobj_master_conformance_e2e)
    endif()

    add_test(NAME pyramid_ada_build_artifacts
        COMMAND ${CMAKE_COMMAND}
                --build ${CMAKE_BINARY_DIR}
                --config $<CONFIG>
                --target pyramid_ada_all)
    set_tests_properties(pyramid_ada_build_artifacts PROPERTIES
        FIXTURES_SETUP pyramid_ada_artifacts
        LABELS "ada;build"
        RESOURCE_LOCK pyramid_ada_build
        TIMEOUT 300)
    if(_pyramid_ada_tests)
        set_tests_properties(${_pyramid_ada_tests} PROPERTIES
            FIXTURES_REQUIRED pyramid_ada_artifacts)
    endif()
endif()

if(TARGET pyramid_grpc_transport)
    add_executable(test_grpc_transport_smoke
        test_grpc_transport_smoke.cpp
    )
    target_link_libraries(test_grpc_transport_smoke PRIVATE
        pyramid_grpc_transport
        pyramid_protobuf_support
        GTest::gtest_main
    )
    target_include_directories(test_grpc_transport_smoke PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp
        ${PYRAMID_PROTOBUF_GEN_DIR}
    )
    gtest_discover_tests(test_grpc_transport_smoke DISCOVERY_MODE PRE_TEST)
endif()

if(TARGET pyramid_grpc_coupled_plugin)
    add_executable(test_grpc_coupled_plugin_load
        test_grpc_coupled_plugin_load.cpp
    )
    add_dependencies(test_grpc_coupled_plugin_load pyramid_grpc_coupled_plugin)
    target_link_libraries(test_grpc_coupled_plugin_load PRIVATE
        pcl_core
        GTest::gtest_main
    )
    target_include_directories(test_grpc_coupled_plugin_load PRIVATE
        ${PYRAMID_TEST_GENERATED_DIR}/$<CONFIG>
    )
    gtest_discover_tests(test_grpc_coupled_plugin_load DISCOVERY_MODE PRE_TEST)
endif()

if(TARGET pyramid_grpc_coupled_plugin AND TARGET pyramid_grpc_transport)
    add_executable(test_grpc_coupled_plugin_e2e
        test_grpc_coupled_plugin_e2e.cpp
    )
    add_dependencies(test_grpc_coupled_plugin_e2e pyramid_grpc_coupled_plugin)
    target_link_libraries(test_grpc_coupled_plugin_e2e PRIVATE
        pcl_core
        pyramid_grpc_transport
        pyramid_protobuf_support
        GTest::gtest_main
    )
    target_include_directories(test_grpc_coupled_plugin_e2e PRIVATE
        ${PYRAMID_TEST_GENERATED_DIR}
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_CPP_BINDINGS_DIR}/grpc/cpp
        ${PYRAMID_PROTOBUF_GEN_DIR}
    )
    gtest_discover_tests(test_grpc_coupled_plugin_e2e DISCOVERY_MODE PRE_TEST)
endif()

if(TARGET pyramid_ros2_transport)
    add_executable(test_ros2_transport_semantics
        test_ros2_transport_semantics.cpp
    )
    target_link_libraries(test_ros2_transport_semantics PRIVATE
        pyramid_ros2_transport
        GTest::gtest_main
    )
    target_include_directories(test_ros2_transport_semantics PRIVATE
        ${PROTO_BINDINGS_DIR}
        ${PYRAMID_CPP_BINDINGS_DIR}/ros2/cpp
    )
    gtest_discover_tests(test_ros2_transport_semantics DISCOVERY_MODE PRE_TEST)
    add_test(NAME tobj_ros2_facade_e2e
        COMMAND $<TARGET_FILE:test_ros2_transport_semantics>
                --gtest_filter=Ros2TransportSemantics.*)
    set_tests_properties(tobj_ros2_facade_e2e PROPERTIES
        LABELS "integration;ros2;e2e;facade"
        TIMEOUT 20)
endif()

get_property(_pyramid_tests_build_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(_pyramid_test_target IN LISTS _pyramid_tests_build_targets)
    if(_pyramid_test_target MATCHES "^test_"
            AND NOT _pyramid_test_target STREQUAL "test_pcl_codec_registry"
            AND NOT _pyramid_test_target STREQUAL "test_pcl_plugin_loader")
        target_sources(${_pyramid_test_target} PRIVATE
            $<TARGET_OBJECTS:pyramid_cpp_test_codec_plugins>)
        target_link_libraries(${_pyramid_test_target} PRIVATE
            pcl_core)
    endif()
endforeach()
unset(_pyramid_tests_build_targets)
unset(_pyramid_test_target)
