include(GoogleTest)

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

# --- Integration test ---
add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE mujin_core GTest::gtest_main)
gtest_discover_tests(test_integration)
