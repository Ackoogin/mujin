include(GoogleTest)

set(TOBJ_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tactical_objects)
set(PROTO_BINDINGS_DIR ${PYRAMID_ROOT_DIR}/bindings/cpp/generated)

find_program(PYRAMID_PYTHON NAMES python3 python)
if(NOT PYRAMID_PYTHON)
    message(FATAL_ERROR "Python is required for PYRAMID binding generator tests")
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

add_executable(tobj_socket_server
    ${TOBJ_TEST_DIR}/tobj_socket_server.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_provided.cpp
    ${PROTO_BINDINGS_DIR}/pyramid_services_tactical_objects_consumed.cpp
)
target_link_libraries(tobj_socket_server PRIVATE
    tactical_objects tactical_objects_component
    pcl_core pcl_transport_socket
    pyramid_generated_codecs
    pyramid_flatbuffers_support
)
target_include_directories(tobj_socket_server PRIVATE
    ${PCL_ROOT_DIR}/src
    ${PROTO_BINDINGS_DIR}
)
if(NOT WIN32)
    target_link_libraries(tobj_socket_server PRIVATE pthread)
endif()

add_executable(standalone_bridge
    ${TOBJ_TEST_DIR}/standalone_bridge.cpp
)
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

if(WIN32)
    add_test(NAME tobj_cpp_bridge_client_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.bat
                -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    add_test(NAME tobj_cpp_bridge_client_flatbuffers_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.bat
                -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -DSCRIPT_ARGS=--content-type|application/flatbuffers
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
else()
    add_test(NAME tobj_cpp_bridge_client_e2e
        COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.sh
                --server-bin $<TARGET_FILE:tobj_socket_server>
                --bridge-bin $<TARGET_FILE:standalone_bridge>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>)
    add_test(NAME tobj_cpp_bridge_client_flatbuffers_e2e
        COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.sh
                --server-bin $<TARGET_FILE:tobj_socket_server>
                --bridge-bin $<TARGET_FILE:standalone_bridge>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>
                --content-type application/flatbuffers)
endif()
set_tests_properties(
    tobj_cpp_bridge_client_e2e
    tobj_cpp_bridge_client_flatbuffers_e2e
    PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 30
        RESOURCE_LOCK pyramid_process_e2e)

if(WIN32)
    add_test(NAME tobj_cpp_app_client_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    add_test(NAME tobj_cpp_app_client_flatbuffers_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -DSCRIPT_ARGS=--content-type|application/flatbuffers
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    add_test(NAME tobj_cpp_app_client_protobuf_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -DSCRIPT_ARGS=--content-type|application/protobuf
                -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
else()
    add_test(NAME tobj_cpp_app_client_e2e
        COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                --app-bin $<TARGET_FILE:tactical_objects_app>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>)
    add_test(NAME tobj_cpp_app_client_flatbuffers_e2e
        COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                --app-bin $<TARGET_FILE:tactical_objects_app>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>
                --content-type application/flatbuffers)
    add_test(NAME tobj_cpp_app_client_protobuf_e2e
        COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.sh
                --app-bin $<TARGET_FILE:tactical_objects_app>
                --client-bin $<TARGET_FILE:tactical_objects_test_client>
                --content-type application/protobuf)
endif()
set_tests_properties(
    tobj_cpp_app_client_e2e
    tobj_cpp_app_client_flatbuffers_e2e
    tobj_cpp_app_client_protobuf_e2e
    PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 30
        RESOURCE_LOCK pyramid_process_e2e)

if(WIN32 AND TARGET pyramid_grpc_transport)
    add_executable(tobj_grpc_server
        tobj_grpc_server.cpp
    )
    target_link_libraries(tobj_grpc_server PRIVATE
        pyramid_grpc_transport
        pyramid_protobuf_support
    )
    target_include_directories(tobj_grpc_server PRIVATE
        ${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp
        ${PYRAMID_PROTOBUF_GEN_DIR}
    )
endif()

find_program(GPRBUILD gprbuild)
set(PYRAMID_PROTO_ROOT ${PYRAMID_ROOT_DIR}/proto)
if(GPRBUILD AND EXISTS "${PYRAMID_PROTO_ROOT}")
    set(ADA_TEST_SRC_DIR ${PYRAMID_ROOT_DIR}/tests/ada)
    find_program(ADA_PYTHON NAMES python3 python)
    if(NOT ADA_PYTHON)
        message(FATAL_ERROR "Python is required to generate Ada bindings for tests")
    endif()
    set(BINDINGS_GEN_SCRIPT ${PYRAMID_ROOT_DIR}/pim/generate_bindings.py)
    set(ADA_GEN_OUT ${PYRAMID_ROOT_DIR}/bindings/ada/generated)
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
        )
        set(_ada_fb_mkdir COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PYRAMID_GEN_LIB_DIR})

    add_custom_target(ada_test_generated_bindings_build ALL
        COMMAND ${ADA_PYTHON} ${BINDINGS_GEN_SCRIPT} ${PYRAMID_PROTO_ROOT} ${ADA_GEN_OUT} --languages ada --backends json,flatbuffers,protobuf,grpc
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/protobuf/ada/pyramid_components_tactical_objects-protobuf_codec.ads
                ${ADA_GEN_OUT}/protobuf/ada/pyramid-components-tactical_objects-protobuf_codec.ads
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/protobuf/ada/pyramid_data_model_base-protobuf_codec.ads
                ${ADA_GEN_OUT}/protobuf/ada/pyramid-data_model-base-protobuf_codec.ads
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/protobuf/ada/pyramid_data_model_common-protobuf_codec.ads
                ${ADA_GEN_OUT}/protobuf/ada/pyramid-data_model-common-protobuf_codec.ads
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/protobuf/ada/pyramid_data_model_tactical-protobuf_codec.ads
                ${ADA_GEN_OUT}/protobuf/ada/pyramid-data_model-tactical-protobuf_codec.ads
        # Note: grpc backend now generates files with correct dash-separated names directly,
        # along with required parent package stubs - no copy/rename needed
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_TEST_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PCL_LIB_DIR}
        ${_ada_fb_mkdir}
        COMMAND ${CMAKE_COMMAND} -E env UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                ${GPRBUILD} -p -P ${ADA_TEST_SRC_DIR}/test_generated_bindings.gpr
                -XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                -XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR}
                -XPCL_LIB_DIR=${ADA_PCL_LIB_DIR}
                -XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}
                -XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME}
                ${_ada_fb_args}
        WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
        COMMENT "Building Ada generated-bindings test"
    )
    if(WIN32)
        add_custom_command(TARGET ada_test_generated_bindings_build PRE_BUILD
            COMMAND cmd /c "${ADA_PCL_BUILD_SCRIPT}" "${ADA_PCL_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible PCL static archives for Ada builds")
    endif()
    if(WIN32)
        add_custom_command(TARGET ada_test_generated_bindings_build PRE_BUILD
            COMMAND cmd /c "${ADA_PYRAMID_GEN_BUILD_SCRIPT}" "${ADA_PYRAMID_GEN_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Ada builds")
    else()
        add_custom_command(TARGET ada_test_generated_bindings_build PRE_BUILD
            COMMAND bash "${ADA_PYRAMID_GEN_BUILD_SCRIPT}" "${ADA_PYRAMID_GEN_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Ada builds")
    endif()

    add_test(NAME ada_generated_bindings_roundtrip
        COMMAND ${ADA_TEST_SRC_DIR}/bin/test_generated_bindings)
    set_tests_properties(ada_generated_bindings_roundtrip PROPERTIES
        LABELS "ada;generated;codec"
        TIMEOUT 20
        DEPENDS ada_test_generated_bindings_build)

    if(WIN32)
        add_test(NAME tobj_ada_socket_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_socket_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_tobj_client
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_socket_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_socket_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_tobj_client)
    endif()
    set_tests_properties(tobj_ada_socket_e2e PROPERTIES
        LABELS "cross_language;integration"
        TIMEOUT 60
        RESOURCE_LOCK pyramid_process_e2e)

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
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e)
    endif()
    set_tests_properties(tobj_ada_active_find_e2e PROPERTIES
        LABELS "cross_language;integration"
        TIMEOUT 40
        RESOURCE_LOCK pyramid_process_e2e)

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
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    --content-type application/flatbuffers)
    endif()
    set_tests_properties(tobj_ada_active_find_flatbuffers_e2e PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 40
        RESOURCE_LOCK pyramid_process_e2e)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_app_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_app_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e)
    endif()
    set_tests_properties(tobj_ada_active_find_app_e2e PROPERTIES
        LABELS "cross_language;integration;real_app"
        TIMEOUT 40
        RESOURCE_LOCK pyramid_process_e2e)

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
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e
                    --content-type application/flatbuffers)
    endif()
    set_tests_properties(tobj_ada_active_find_app_flatbuffers_e2e PROPERTIES
        LABELS "cross_language;integration;codec;real_app"
        TIMEOUT 40
        RESOURCE_LOCK pyramid_process_e2e)

    # -- Pyramid Bridge Ada build + E2E test -----------------------------------
    set(ADA_BRIDGE_SRC_DIR ${PYRAMID_ROOT_DIR}/pyramid_bridge/ada)
    set(ADA_PCL_SHMEM_LIB_NAME "pcl_transport_shared_memory" CACHE STRING
        "Base library name for Ada gprbuild PCL shared-memory transport static library")

    add_custom_target(pyramid_bridge_ada_build ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_BRIDGE_SRC_DIR}/bin
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PCL_LIB_DIR}
        ${_ada_fb_mkdir}
        COMMAND ${CMAKE_COMMAND} -E env UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                ${GPRBUILD} -p -P ${ADA_BRIDGE_SRC_DIR}/pyramid_bridge.gpr -q
                -XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                -XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR}
                -XPCL_LIB_DIR=${ADA_PCL_LIB_DIR}
                -XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}
                -XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME}
                -XPCL_SHMEM_LIB_NAME=${ADA_PCL_SHMEM_LIB_NAME}
                ${_ada_fb_args}
        WORKING_DIRECTORY ${ADA_BRIDGE_SRC_DIR}
        COMMENT "Building Ada Pyramid Bridge"
        DEPENDS ada_test_generated_bindings_build
    )
    if(WIN32)
        add_custom_command(TARGET pyramid_bridge_ada_build PRE_BUILD
            COMMAND cmd /c "${ADA_PCL_BUILD_SCRIPT}" "${ADA_PCL_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible PCL static archives for Pyramid Bridge build")
    endif()
    if(WIN32)
        add_custom_command(TARGET pyramid_bridge_ada_build PRE_BUILD
            COMMAND cmd /c "${ADA_PYRAMID_GEN_BUILD_SCRIPT}" "${ADA_PYRAMID_GEN_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Pyramid Bridge build")
    else()
        add_custom_command(TARGET pyramid_bridge_ada_build PRE_BUILD
            COMMAND bash "${ADA_PYRAMID_GEN_BUILD_SCRIPT}" "${ADA_PYRAMID_GEN_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Pyramid Bridge build")
    endif()

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
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_pyramid_bridge_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --stub-bin $<TARGET_FILE:ame_backend_stub>
                    --evidence-bin $<TARGET_FILE:pyramid_bridge_evidence_client>
                    --bridge-bin ${PYRAMID_ROOT_DIR}/pyramid_bridge/ada/bin/pyramid_bridge_main)
    endif()
    set_tests_properties(pyramid_bridge_e2e PROPERTIES
        LABELS "cross_language;integration;shared_memory;real_app"
        TIMEOUT 60
        RESOURCE_LOCK pyramid_process_e2e)

    endif() # PYRAMID_ENABLE_FLATBUFFERS

    if(WIN32 AND TARGET pyramid_grpc_transport)
        add_custom_target(ada_grpc_cpp_interop_build ALL
            COMMAND ${CMAKE_COMMAND} -E env UNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                    ${GPRBUILD} -p -P ${ADA_TEST_SRC_DIR}/ada_grpc_cpp_interop_e2e.gpr
                    -XUNMANNED_ROOT=${UNMANNED_WORKSPACE_ROOT}
                    -XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR}
                    -XPCL_LIB_DIR=${ADA_PCL_LIB_DIR}
                    -XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}
                    -XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME}
                    -XPYRAMID_GEN_LIB_DIR=${ADA_PYRAMID_GEN_LIB_DIR}
                    -XPYRAMID_GEN_LIB_NAME=${ADA_PYRAMID_GEN_LIB_NAME}
            WORKING_DIRECTORY ${ADA_TEST_SRC_DIR}
            COMMENT "Building Ada gRPC C++ interop test"
        )
        add_dependencies(ada_grpc_cpp_interop_build
            pyramid_grpc_c_shim
            ada_test_generated_bindings_build)

        add_test(NAME tobj_ada_grpc_cpp_interop_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_grpc_cpp_interop_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_grpc_server>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_grpc_cpp_interop_e2e
                    -DSCRIPT_ARGS=--dll-bin|$<TARGET_FILE:pyramid_grpc_c_shim>|--address|127.0.0.1:50111
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        set_tests_properties(tobj_ada_grpc_cpp_interop_e2e PROPERTIES
            LABELS "cross_language;integration;grpc"
            TIMEOUT 30
            RESOURCE_LOCK pyramid_process_e2e)

        add_test(NAME tobj_master_conformance_e2e
                    COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_tobj_master_conformance.bat
                    -DSCRIPT_ARGS=--socket-server-bin|$<TARGET_FILE:tobj_socket_server>|--bridge-bin|$<TARGET_FILE:standalone_bridge>|--socket-client-bin|${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_active_find_e2e|--grpc-server-bin|$<TARGET_FILE:tobj_grpc_server>|--grpc-client-bin|${PYRAMID_ROOT_DIR}/tests/ada/bin/ada_grpc_cpp_interop_e2e|--grpc-dll-bin|$<TARGET_FILE:pyramid_grpc_c_shim>
                    -P ${UNMANNED_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        set_tests_properties(tobj_master_conformance_e2e PROPERTIES
            LABELS "cross_language;integration;conformance;codec;grpc"
            TIMEOUT 120
            RESOURCE_LOCK pyramid_process_e2e)
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
        ${PYRAMID_ROOT_DIR}/bindings/cpp/generated/grpc/cpp
        ${PYRAMID_PROTOBUF_GEN_DIR}
    )
    gtest_discover_tests(test_grpc_transport_smoke DISCOVERY_MODE PRE_TEST)
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
        ${PYRAMID_ROOT_DIR}/bindings/cpp/generated/ros2/cpp
    )
    gtest_discover_tests(test_ros2_transport_semantics DISCOVERY_MODE PRE_TEST)
    add_test(NAME tobj_ros2_facade_e2e
        COMMAND $<TARGET_FILE:test_ros2_transport_semantics>
                --gtest_filter=Ros2TransportSemantics.*)
    set_tests_properties(tobj_ros2_facade_e2e PROPERTIES
        LABELS "integration;ros2;e2e;facade"
        TIMEOUT 20)
endif()
