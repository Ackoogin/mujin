include(GoogleTest)

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

add_executable(test_pcl_streaming test_pcl_streaming.cpp)
target_link_libraries(test_pcl_streaming PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_streaming DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_bridge test_pcl_bridge.cpp)
target_link_libraries(test_pcl_bridge PRIVATE pcl_core GTest::gtest_main)
target_include_directories(test_pcl_bridge PRIVATE ${PCL_ROOT_DIR}/src)
gtest_discover_tests(test_pcl_bridge DISCOVERY_MODE PRE_TEST)

if(NOT MSVC)
    add_executable(test_pcl_oom test_pcl_oom.cpp)
    target_link_libraries(test_pcl_oom PRIVATE pcl_core GTest::gtest_main)
    target_link_options(test_pcl_oom PRIVATE -Wl,--wrap=malloc,--wrap=calloc)
    gtest_discover_tests(test_pcl_oom DISCOVERY_MODE PRE_TEST)
endif()

add_executable(test_pcl_dining test_pcl_dining.cpp)
target_link_libraries(test_pcl_dining PRIVATE pcl_core GTest::gtest_main)
target_include_directories(test_pcl_dining PRIVATE ${PCL_ROOT_DIR}/src)
gtest_discover_tests(test_pcl_dining DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_socket_transport test_pcl_socket_transport.cpp)
target_link_libraries(test_pcl_socket_transport PRIVATE pcl_core pcl_transport_socket GTest::gtest_main)
if(NOT WIN32)
    target_link_libraries(test_pcl_socket_transport PRIVATE pthread)
endif()
gtest_discover_tests(test_pcl_socket_transport DISCOVERY_MODE PRE_TEST)

add_executable(pcl_shm_peer_helper pcl_shm_peer_helper.cpp)
target_link_libraries(pcl_shm_peer_helper PRIVATE pcl_core pcl_transport_shared_memory)
if(NOT WIN32)
    target_link_libraries(pcl_shm_peer_helper PRIVATE pthread)
endif()

add_executable(test_pcl_shared_memory_transport test_pcl_shared_memory_transport.cpp)
target_link_libraries(test_pcl_shared_memory_transport PRIVATE pcl_core pcl_transport_shared_memory GTest::gtest_main)
if(NOT WIN32)
    target_link_libraries(test_pcl_shared_memory_transport PRIVATE pthread)
endif()
add_dependencies(test_pcl_shared_memory_transport pcl_shm_peer_helper)
target_compile_definitions(test_pcl_shared_memory_transport PRIVATE
    PCL_SHM_PEER_HELPER_PATH="$<TARGET_FILE:pcl_shm_peer_helper>"
)
gtest_discover_tests(test_pcl_shared_memory_transport DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_cpp_wrappers test_pcl_cpp_wrappers.cpp)
target_link_libraries(test_pcl_cpp_wrappers PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_cpp_wrappers DISCOVERY_MODE PRE_TEST)
