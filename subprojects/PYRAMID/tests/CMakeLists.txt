include(GoogleTest)

set(TOBJ_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tactical_objects)
set(PROTO_BINDINGS_DIR ${PYRAMID_ROOT_DIR}/examples/cpp/generated)

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

if(PYRAMID_ENABLE_PROTOBUF)
    if(WIN32)
        add_test(NAME generated_bindings_flatbuffers_dispatch
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_generated_codec_dispatch.bat
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME generated_bindings_flatbuffers_dispatch
            COMMAND bash ${PYRAMID_ROOT_DIR}/scripts/test_generated_codec_dispatch.sh)
    endif()
    set_tests_properties(generated_bindings_flatbuffers_dispatch PROPERTIES
        LABELS "generated;codec;dispatch"
        TIMEOUT 120)
endif()

add_executable(tobj_socket_server ${TOBJ_TEST_DIR}/tobj_socket_server.cpp)
target_link_libraries(tobj_socket_server PRIVATE
    tactical_objects tactical_objects_component
    pcl_core pcl_transport_socket
)
target_include_directories(tobj_socket_server PRIVATE ${PCL_ROOT_DIR}/src)
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

if(WIN32)
    add_test(NAME tobj_cpp_bridge_client_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.bat
                -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    add_test(NAME tobj_cpp_bridge_client_flatbuffers_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_bridge_client.bat
                -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -DSCRIPT_ARGS=--content-type|application/flatbuffers
                -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
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
    PROPERTIES LABELS "cross_language;integration;codec" TIMEOUT 30)

if(WIN32)
    add_test(NAME tobj_cpp_app_client_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    add_test(NAME tobj_cpp_app_client_flatbuffers_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -DSCRIPT_ARGS=--content-type|application/flatbuffers
                -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    add_test(NAME tobj_cpp_app_client_protobuf_e2e
        COMMAND ${CMAKE_COMMAND}
                -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_cpp_app_client.bat
                -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                -DCLIENT_BIN=$<TARGET_FILE:tactical_objects_test_client>
                -DSCRIPT_ARGS=--content-type|application/protobuf
                -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
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
    PROPERTIES LABELS "cross_language;integration;codec" TIMEOUT 30)

if(WIN32 AND TARGET pyramid_grpc_transport)
    add_executable(tobj_grpc_server
        tobj_grpc_server.cpp
    )
    target_link_libraries(tobj_grpc_server PRIVATE
        pyramid_grpc_transport
        pyramid_protobuf_support
    )
    target_include_directories(tobj_grpc_server PRIVATE
        ${PYRAMID_ROOT_DIR}/examples/grpc/cpp
        ${PYRAMID_PROTOBUF_GEN_DIR}
    )
endif()

find_program(GPRBUILD gprbuild)
set(PYRAMID_PROTO_ROOT ${PYRAMID_ROOT_DIR}/proto)
if(GPRBUILD AND EXISTS "${PYRAMID_PROTO_ROOT}")
    set(ADA_SRC_DIR ${PYRAMID_ROOT_DIR}/examples/ada)
    find_program(ADA_PYTHON NAMES python3 python)
    if(NOT ADA_PYTHON)
        message(FATAL_ERROR "Python is required to generate Ada bindings for tests")
    endif()
    set(BINDINGS_GEN_SCRIPT ${PYRAMID_ROOT_DIR}/pim/generate_bindings.py)
    set(ADA_GEN_OUT ${ADA_SRC_DIR}/generated)
    set(ADA_PCL_INCLUDE_DIR "${PCL_ROOT_DIR}/include" CACHE PATH
        "Include directory for Ada gprbuild PCL headers")
    set(ADA_PCL_LIB_NAME "pcl_core" CACHE STRING
        "Base library name for Ada gprbuild core PCL static library")
    set(ADA_PCL_SOCKET_LIB_NAME "pcl_transport_socket" CACHE STRING
        "Base library name for Ada gprbuild PCL socket transport static library")
    set(ADA_PYRAMID_GEN_LIB_NAME "pyramid_generated_flatbuffers_codec" CACHE STRING
        "Base library name for Ada gprbuild generated FlatBuffers codec archive")

    if(WIN32)
        set(ADA_PCL_LIB_DIR "${CMAKE_BINARY_DIR}/ada_gnat_pcl" CACHE PATH
            "Directory containing GNAT-compatible libpcl_core.a and libpcl_transport_socket.a")
        set(ADA_PCL_BUILD_SCRIPT "${PCL_ROOT_DIR}/scripts/build_gnat_pcl_static_libs.bat")
        set(ADA_PYRAMID_GEN_LIB_DIR "${CMAKE_BINARY_DIR}/ada_gnat_pyramid" CACHE PATH
            "Directory containing GNAT-compatible generated FlatBuffers codec archive")
        set(ADA_PYRAMID_GEN_BUILD_SCRIPT "${PYRAMID_ROOT_DIR}/scripts/build_gnat_generated_flatbuffers_libs.bat")
    else()
        set(ADA_PCL_LIB_DIR "${CMAKE_BINARY_DIR}/subprojects/PCL/src" CACHE PATH
            "Directory containing libpcl_core/libpcl_transport_socket for Ada gprbuild")
        set(ADA_PYRAMID_GEN_LIB_DIR "${CMAKE_BINARY_DIR}/ada_gnat_pyramid" CACHE PATH
            "Directory containing GNAT-compatible generated FlatBuffers codec archive")
        set(ADA_PYRAMID_GEN_BUILD_SCRIPT "${PYRAMID_ROOT_DIR}/scripts/build_gnat_generated_flatbuffers_libs.sh")
    endif()

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
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/grpc/ada/pyramid_components_tactical_objects-grpc_transport.ads
                ${ADA_GEN_OUT}/grpc/ada/pyramid-components-tactical_objects-grpc_transport.ads
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/grpc/ada/pyramid_components_tactical_objects_services_consumed-grpc_transport.ads
                ${ADA_GEN_OUT}/grpc/ada/pyramid-components-tactical_objects-services-consumed-grpc_transport.ads
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ADA_GEN_OUT}/grpc/ada/pyramid_components_tactical_objects_services_provided-grpc_transport.ads
                ${ADA_GEN_OUT}/grpc/ada/pyramid-components-tactical_objects-services-provided-grpc_transport.ads
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PCL_LIB_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ADA_PYRAMID_GEN_LIB_DIR}
        COMMAND ${CMAKE_COMMAND} -E env MUJIN_ROOT=${MUJIN_WORKSPACE_ROOT}
                ${GPRBUILD} -p -P ${ADA_SRC_DIR}/test_generated_bindings.gpr
                -XMUJIN_ROOT=${MUJIN_WORKSPACE_ROOT}
                -XPCL_INCLUDE_DIR=${ADA_PCL_INCLUDE_DIR}
                -XPCL_LIB_DIR=${ADA_PCL_LIB_DIR}
                -XPCL_LIB_NAME=${ADA_PCL_LIB_NAME}
                -XPCL_SOCKET_LIB_NAME=${ADA_PCL_SOCKET_LIB_NAME}
                -XPYRAMID_GEN_LIB_DIR=${ADA_PYRAMID_GEN_LIB_DIR}
                -XPYRAMID_GEN_LIB_NAME=${ADA_PYRAMID_GEN_LIB_NAME}
        WORKING_DIRECTORY ${ADA_SRC_DIR}
        COMMENT "Building Ada generated-bindings test"
    )
    if(WIN32)
        add_custom_command(TARGET ada_test_generated_bindings_build PRE_BUILD
            COMMAND cmd /c "${ADA_PCL_BUILD_SCRIPT}" "${ADA_PCL_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible PCL static archives for Ada builds")
        add_custom_command(TARGET ada_test_generated_bindings_build PRE_BUILD
            COMMAND cmd /c "${ADA_PYRAMID_GEN_BUILD_SCRIPT}" "${ADA_PYRAMID_GEN_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Ada builds")
    else()
        add_custom_command(TARGET ada_test_generated_bindings_build PRE_BUILD
            COMMAND bash "${ADA_PYRAMID_GEN_BUILD_SCRIPT}" "${ADA_PYRAMID_GEN_LIB_DIR}"
            COMMENT "Preparing GNAT-compatible generated FlatBuffers archive for Ada builds")
    endif()

    add_test(NAME ada_generated_bindings_roundtrip
        COMMAND ${ADA_SRC_DIR}/bin/test_generated_bindings)
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
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_socket_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_socket_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_tobj_client)
    endif()
    set_tests_properties(tobj_ada_socket_e2e PROPERTIES
        LABELS "cross_language;integration"
        TIMEOUT 60)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e)
    endif()
    set_tests_properties(tobj_ada_active_find_e2e PROPERTIES
        LABELS "cross_language;integration"
        TIMEOUT 40)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_flatbuffers_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_socket_server>
                    -DBRIDGE_BIN=$<TARGET_FILE:standalone_bridge>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e
                    -DSCRIPT_ARGS=--content-type|application/flatbuffers
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_flatbuffers_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_e2e.sh
                    --server-bin $<TARGET_FILE:tobj_socket_server>
                    --bridge-bin $<TARGET_FILE:standalone_bridge>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e
                    --content-type application/flatbuffers)
    endif()
    set_tests_properties(tobj_ada_active_find_flatbuffers_e2e PROPERTIES
        LABELS "cross_language;integration;codec"
        TIMEOUT 40)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_app_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_app_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e)
    endif()
    set_tests_properties(tobj_ada_active_find_app_e2e PROPERTIES
        LABELS "cross_language;integration;real_app"
        TIMEOUT 40)

    if(WIN32)
        add_test(NAME tobj_ada_active_find_app_flatbuffers_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.bat
                    -DAPP_BIN=$<TARGET_FILE:tactical_objects_app>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e
                    -DSCRIPT_ARGS=--content-type|application/flatbuffers
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
    else()
        add_test(NAME tobj_ada_active_find_app_flatbuffers_e2e
            COMMAND ${PYRAMID_ROOT_DIR}/scripts/test_ada_active_find_app_e2e.sh
                    --app-bin $<TARGET_FILE:tactical_objects_app>
                    --client-bin ${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e
                    --content-type application/flatbuffers)
    endif()
    set_tests_properties(tobj_ada_active_find_app_flatbuffers_e2e PROPERTIES
        LABELS "cross_language;integration;codec;real_app"
        TIMEOUT 40)

    if(WIN32 AND TARGET pyramid_grpc_transport)
        add_custom_target(ada_grpc_cpp_interop_build ALL
            COMMAND ${CMAKE_COMMAND} -E env MUJIN_ROOT=${MUJIN_WORKSPACE_ROOT}
                    ${GPRBUILD} -p -P ${ADA_SRC_DIR}/ada_grpc_cpp_interop_e2e.gpr
            WORKING_DIRECTORY ${ADA_SRC_DIR}
            COMMENT "Building Ada gRPC C++ interop test"
        )
        add_dependencies(ada_grpc_cpp_interop_build pyramid_grpc_c_shim)

        add_test(NAME tobj_ada_grpc_cpp_interop_e2e
            COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_ada_grpc_cpp_interop_e2e.bat
                    -DSERVER_BIN=$<TARGET_FILE:tobj_grpc_server>
                    -DCLIENT_BIN=${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_grpc_cpp_interop_e2e
                    -DSCRIPT_ARGS=--dll-bin|$<TARGET_FILE:pyramid_grpc_c_shim>|--address|127.0.0.1:50111
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        set_tests_properties(tobj_ada_grpc_cpp_interop_e2e PROPERTIES
            LABELS "cross_language;integration;grpc"
            TIMEOUT 30)

        add_test(NAME tobj_master_conformance_e2e
                    COMMAND ${CMAKE_COMMAND}
                    -DSCRIPT=${PYRAMID_ROOT_DIR}/scripts/test_tobj_master_conformance.bat
                    -DSCRIPT_ARGS=--socket-server-bin|$<TARGET_FILE:tobj_socket_server>|--bridge-bin|$<TARGET_FILE:standalone_bridge>|--socket-client-bin|${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_active_find_e2e|--grpc-server-bin|$<TARGET_FILE:tobj_grpc_server>|--grpc-client-bin|${PYRAMID_ROOT_DIR}/examples/ada/bin/ada_grpc_cpp_interop_e2e|--grpc-dll-bin|$<TARGET_FILE:pyramid_grpc_c_shim>
                    -P ${MUJIN_WORKSPACE_ROOT}/cmake/run_bat_test.cmake)
        set_tests_properties(tobj_master_conformance_e2e PROPERTIES
            LABELS "cross_language;integration;conformance;codec;grpc"
            TIMEOUT 120)
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
        ${PYRAMID_ROOT_DIR}/examples/grpc/cpp
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
        ${PYRAMID_ROOT_DIR}/examples/ros2/cpp
    )
    gtest_discover_tests(test_ros2_transport_semantics DISCOVERY_MODE PRE_TEST)
endif()
