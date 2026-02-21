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
