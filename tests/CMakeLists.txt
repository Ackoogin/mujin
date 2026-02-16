include(GoogleTest)

# Make source dir available for file-based tests
add_compile_definitions(PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# --- Unit tests ---
add_executable(test_world_model test_world_model.cpp)
target_link_libraries(test_world_model PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_world_model)

add_executable(test_action_registry test_action_registry.cpp)
target_link_libraries(test_action_registry PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_action_registry)

add_executable(test_plan_compiler test_plan_compiler.cpp)
target_link_libraries(test_plan_compiler PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_plan_compiler)

add_executable(test_pddl_parser test_pddl_parser.cpp)
target_link_libraries(test_pddl_parser PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_pddl_parser)

# --- Integration test ---
add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_integration)
