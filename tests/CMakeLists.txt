include(GoogleTest)

# Make source dir available for file-based tests (only for parser test)
set(AME_SOURCE_DIR "${CMAKE_SOURCE_DIR}")

# --- Unit tests ---
add_executable(test_world_model test_world_model.cpp)
target_link_libraries(test_world_model PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_world_model DISCOVERY_MODE PRE_TEST)

add_executable(test_action_registry test_action_registry.cpp)
target_link_libraries(test_action_registry PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_action_registry DISCOVERY_MODE PRE_TEST)

add_executable(test_plan_compiler test_plan_compiler.cpp)
target_link_libraries(test_plan_compiler PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_plan_compiler DISCOVERY_MODE PRE_TEST)

add_executable(test_pddl_parser test_pddl_parser.cpp)
target_link_libraries(test_pddl_parser PRIVATE ame_core GTest::gtest_main)
target_compile_definitions(test_pddl_parser PRIVATE "PROJECT_SOURCE_DIR=\"${AME_SOURCE_DIR}\"")
gtest_discover_tests(test_pddl_parser DISCOVERY_MODE PRE_TEST)

add_executable(test_planner test_planner.cpp)
target_link_libraries(test_planner PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_planner DISCOVERY_MODE PRE_TEST)

# --- Integration tests ---
add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_integration DISCOVERY_MODE PRE_TEST)

# --- End-to-end pipeline test ---
add_executable(test_e2e_pipeline test_e2e_pipeline.cpp)
target_link_libraries(test_e2e_pipeline PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_e2e_pipeline DISCOVERY_MODE PRE_TEST)

# --- Observability tests (Layers 1-3) ---
add_executable(test_observability test_observability.cpp)
target_link_libraries(test_observability PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_observability DISCOVERY_MODE PRE_TEST)

# --- Core extensions tests (Extensions 3-6) ---
add_executable(test_extensions test_extensions.cpp)
target_link_libraries(test_extensions PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_extensions DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# PCL (PYRAMID Container Library) tests
# ---------------------------------------------------------------------------
add_executable(test_pcl_lifecycle test_pcl_lifecycle.cpp)
target_link_libraries(test_pcl_lifecycle PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_lifecycle DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_executor test_pcl_executor.cpp)
target_link_libraries(test_pcl_executor PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_executor DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_log test_pcl_log.cpp)
target_link_libraries(test_pcl_log PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_log DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_robustness test_pcl_robustness.cpp)
target_link_libraries(test_pcl_robustness PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_robustness DISCOVERY_MODE PRE_TEST)

if(NOT MSVC)
  add_executable(test_pcl_oom test_pcl_oom.cpp)
  target_link_libraries(test_pcl_oom PRIVATE pcl_core GTest::gtest_main)
  target_link_options(test_pcl_oom PRIVATE -Wl,--wrap=malloc,--wrap=calloc)
  gtest_discover_tests(test_pcl_oom DISCOVERY_MODE PRE_TEST)
endif()

add_executable(test_pcl_dining test_pcl_dining.cpp)
target_link_libraries(test_pcl_dining PRIVATE pcl_core GTest::gtest_main)
# Expose internals so the port-overflow branch in pcl_bridge.c can be tested.
target_include_directories(test_pcl_dining PRIVATE ${CMAKE_SOURCE_DIR}/src/pcl)
gtest_discover_tests(test_pcl_dining DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# PCL-backed Ame component tests
# ---------------------------------------------------------------------------
add_executable(test_world_model_component test_world_model_component.cpp)
target_link_libraries(test_world_model_component PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_world_model_component DISCOVERY_MODE PRE_TEST)

add_executable(test_planner_component test_planner_component.cpp)
target_link_libraries(test_planner_component PRIVATE ame_core GTest::gtest_main)
target_include_directories(test_planner_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_discover_tests(test_planner_component DISCOVERY_MODE PRE_TEST)

add_executable(test_executor_component test_executor_component.cpp)
target_link_libraries(test_executor_component PRIVATE ame_core GTest::gtest_main)
target_include_directories(test_executor_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_discover_tests(test_executor_component DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# Tactical Objects tests
# ---------------------------------------------------------------------------
set(TOBJ_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tactical_objects)

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
target_include_directories(test_tobj_component PRIVATE ${CMAKE_SOURCE_DIR}/src/pcl)
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

# ---------------------------------------------------------------------------
# Streaming optimisation tests (Steps 1-3, 5)
# ---------------------------------------------------------------------------
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
target_link_libraries(test_tobj_e2e PRIVATE tactical_objects tactical_objects_component
  pcl_core GTest::gtest_main)
target_include_directories(test_tobj_e2e PRIVATE ${CMAKE_SOURCE_DIR}/src/pcl)
gtest_discover_tests(test_tobj_e2e DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# Socket transport E2E tests (Step 6: C++ only, no Ada dependency)
# ---------------------------------------------------------------------------
add_executable(test_tobj_socket_e2e ${TOBJ_TEST_DIR}/Test_TacticalObjects_SocketE2E.cpp)
target_link_libraries(test_tobj_socket_e2e PRIVATE
  tactical_objects tactical_objects_component
  pcl_core pcl_transport_socket GTest::gtest_main)
target_include_directories(test_tobj_socket_e2e PRIVATE ${CMAKE_SOURCE_DIR}/src/pcl)
if(NOT WIN32)
  target_link_libraries(test_tobj_socket_e2e PRIVATE pthread)
endif()
gtest_discover_tests(test_tobj_socket_e2e DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# Socket server harness for cross-process Ada E2E (Step 4)
# ---------------------------------------------------------------------------
add_executable(tobj_socket_server ${TOBJ_TEST_DIR}/tobj_socket_server.cpp)
target_link_libraries(tobj_socket_server PRIVATE
  tactical_objects tactical_objects_component
  pcl_core pcl_transport_socket)
target_include_directories(tobj_socket_server PRIVATE ${CMAKE_SOURCE_DIR}/src/pcl)
if(NOT WIN32)
  target_link_libraries(tobj_socket_server PRIVATE pthread)
endif()

# ---------------------------------------------------------------------------
# Cross-process Ada socket E2E test (gated on GNAT availability)
# ---------------------------------------------------------------------------
find_program(GPRBUILD gprbuild)
if(GPRBUILD)
  add_test(NAME tobj_ada_socket_e2e
           COMMAND ${CMAKE_SOURCE_DIR}/scripts/test_ada_socket_e2e.sh
                   --server-bin $<TARGET_FILE:tobj_socket_server>
                   --client-bin ${CMAKE_SOURCE_DIR}/examples/ada/bin/ada_tobj_client)
  set_tests_properties(tobj_ada_socket_e2e PROPERTIES
    LABELS "cross_language;integration"
    TIMEOUT 30)
endif()
