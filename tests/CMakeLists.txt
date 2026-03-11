include(GoogleTest)

# Make source dir available for file-based tests (only for parser test)
set(MUJIN_SOURCE_DIR "${CMAKE_SOURCE_DIR}")

# --- Unit tests ---
add_executable(test_world_model test_world_model.cpp)
target_link_libraries(test_world_model PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_world_model DISCOVERY_MODE PRE_TEST)

add_executable(test_action_registry test_action_registry.cpp)
target_link_libraries(test_action_registry PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_action_registry DISCOVERY_MODE PRE_TEST)

add_executable(test_plan_compiler test_plan_compiler.cpp)
target_link_libraries(test_plan_compiler PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_plan_compiler DISCOVERY_MODE PRE_TEST)

add_executable(test_pddl_parser test_pddl_parser.cpp)
target_link_libraries(test_pddl_parser PRIVATE mujin_core GTest::gtest_main)
target_compile_definitions(test_pddl_parser PRIVATE "PROJECT_SOURCE_DIR=\"${MUJIN_SOURCE_DIR}\"")
gtest_discover_tests(test_pddl_parser DISCOVERY_MODE PRE_TEST)

add_executable(test_planner test_planner.cpp)
target_link_libraries(test_planner PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_planner DISCOVERY_MODE PRE_TEST)

# --- Integration tests ---
add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_integration DISCOVERY_MODE PRE_TEST)

# --- End-to-end pipeline test ---
add_executable(test_e2e_pipeline test_e2e_pipeline.cpp)
target_link_libraries(test_e2e_pipeline PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_e2e_pipeline DISCOVERY_MODE PRE_TEST)

# --- Observability tests (Layers 1-3) ---
add_executable(test_observability test_observability.cpp)
target_link_libraries(test_observability PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_observability DISCOVERY_MODE PRE_TEST)

# --- Core extensions tests (Extensions 3-6) ---
add_executable(test_extensions test_extensions.cpp)
target_link_libraries(test_extensions PRIVATE mujin_core GTest::gtest_main)
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

add_executable(test_pcl_oom test_pcl_oom.cpp)
target_link_libraries(test_pcl_oom PRIVATE pcl_core GTest::gtest_main)
target_link_options(test_pcl_oom PRIVATE -Wl,--wrap=malloc,--wrap=calloc)
gtest_discover_tests(test_pcl_oom DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# PCL-backed Mujin component tests
# ---------------------------------------------------------------------------
add_executable(test_world_model_component test_world_model_component.cpp)
target_link_libraries(test_world_model_component PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_world_model_component DISCOVERY_MODE PRE_TEST)

add_executable(test_planner_component test_planner_component.cpp)
target_link_libraries(test_planner_component PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_planner_component DISCOVERY_MODE PRE_TEST)

add_executable(test_executor_component test_executor_component.cpp)
target_link_libraries(test_executor_component PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_executor_component DISCOVERY_MODE PRE_TEST)

