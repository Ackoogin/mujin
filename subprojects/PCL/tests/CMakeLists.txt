include(GoogleTest)

add_executable(test_pcl_alloc test_pcl_alloc.cpp)
target_link_libraries(test_pcl_alloc PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_alloc DISCOVERY_MODE PRE_TEST)

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

add_executable(test_pcl_codec_registry test_pcl_codec_registry.cpp)
target_link_libraries(test_pcl_codec_registry PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_codec_registry DISCOVERY_MODE PRE_TEST)

add_library(pcl_transport_capture_plugin MODULE pcl_transport_capture_plugin.c)
target_link_libraries(pcl_transport_capture_plugin PRIVATE pcl_core)

add_library(pcl_codec_stub_plugin MODULE pcl_codec_stub_plugin.c)
target_link_libraries(pcl_codec_stub_plugin PRIVATE pcl_core)

add_library(pcl_codec_badabi_plugin MODULE pcl_codec_badabi_plugin.c)
target_link_libraries(pcl_codec_badabi_plugin PRIVATE pcl_core)

add_executable(test_pcl_plugin_loader test_pcl_plugin_loader.cpp)
target_link_libraries(test_pcl_plugin_loader PRIVATE pcl_core GTest::gtest_main)
target_compile_definitions(test_pcl_plugin_loader PRIVATE
    CAPTURE_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_capture_plugin>"
    STUB_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_stub_plugin>"
    BADABI_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_badabi_plugin>"
    SOCKET_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_socket_plugin>"
    SHM_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_shared_memory_plugin>"
    UDP_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_udp_plugin>"
    WRONGABI_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_wrongabi_plugin>"
    NOENTRY_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_noentry_plugin>"
)
add_dependencies(test_pcl_plugin_loader
    pcl_transport_capture_plugin
    pcl_codec_stub_plugin
    pcl_codec_badabi_plugin
    pcl_transport_socket_plugin
    pcl_transport_shared_memory_plugin
    pcl_transport_udp_plugin
    pcl_transport_wrongabi_plugin
    pcl_transport_noentry_plugin
)
gtest_discover_tests(test_pcl_plugin_loader DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_transport_routing test_pcl_transport_routing.cpp)
target_link_libraries(test_pcl_transport_routing PRIVATE pcl_core GTest::gtest_main)
# Internal headers so the rollback test can assert the executor's route table is
# empty after a failed multi-line load.
target_include_directories(test_pcl_transport_routing PRIVATE ${PCL_ROOT_DIR}/src)
target_compile_definitions(test_pcl_transport_routing PRIVATE
    CAPTURE_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_capture_plugin>"
    UDP_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_udp_plugin>"
    SHM_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_shared_memory_plugin>"
    NOCAPS_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_nocaps_plugin>"
)
add_dependencies(test_pcl_transport_routing
    pcl_transport_capture_plugin
    pcl_transport_udp_plugin
    pcl_transport_shared_memory_plugin
    pcl_transport_nocaps_plugin
)
gtest_discover_tests(test_pcl_transport_routing DISCOVERY_MODE PRE_TEST)

add_library(pcl_transport_nocaps_plugin MODULE pcl_transport_nocaps_plugin.c)
target_link_libraries(pcl_transport_nocaps_plugin PRIVATE pcl_core)

add_library(pcl_transport_wrongabi_plugin MODULE pcl_transport_wrongabi_plugin.c)
target_link_libraries(pcl_transport_wrongabi_plugin PRIVATE pcl_core)

add_library(pcl_transport_noentry_plugin MODULE pcl_transport_noentry_plugin.c)
target_link_libraries(pcl_transport_noentry_plugin PRIVATE pcl_core)

add_library(pcl_codec_sequence_plugin MODULE pcl_codec_sequence_plugin.c)
target_link_libraries(pcl_codec_sequence_plugin PRIVATE pcl_core)

add_library(pcl_codec_ports_plugin MODULE pcl_codec_ports_plugin.c)
target_link_libraries(pcl_codec_ports_plugin PRIVATE pcl_core)

add_library(pcl_codec_ports2_plugin MODULE pcl_codec_ports2_plugin.c)
target_link_libraries(pcl_codec_ports2_plugin PRIVATE pcl_core)

add_executable(test_pcl_capabilities test_pcl_capabilities.cpp)
target_link_libraries(test_pcl_capabilities PRIVATE pcl_core GTest::gtest_main)
target_compile_definitions(test_pcl_capabilities PRIVATE
    CAPTURE_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_capture_plugin>"
    NOCAPS_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_nocaps_plugin>"
    UDP_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_udp_plugin>"
    SOCKET_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_socket_plugin>"
    SHM_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_shared_memory_plugin>"
)
add_dependencies(test_pcl_capabilities
    pcl_transport_capture_plugin
    pcl_transport_nocaps_plugin
    pcl_transport_udp_plugin
    pcl_transport_socket_plugin
    pcl_transport_shared_memory_plugin
)
gtest_discover_tests(test_pcl_capabilities DISCOVERY_MODE PRE_TEST)

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

if(NOT MSVC)
    add_executable(test_pcl_socket_faults test_pcl_socket_faults.cpp)
    target_link_libraries(test_pcl_socket_faults PRIVATE pcl_core pcl_transport_socket GTest::gtest_main)
    target_include_directories(test_pcl_socket_faults PRIVATE ${PCL_ROOT_DIR}/src)
    target_link_options(test_pcl_socket_faults PRIVATE
        -Wl,--wrap=malloc,--wrap=calloc)
    if(NOT WIN32)
        target_link_libraries(test_pcl_socket_faults PRIVATE pthread)
    endif()
    gtest_discover_tests(test_pcl_socket_faults DISCOVERY_MODE PRE_TEST)
endif()

add_executable(test_pcl_udp_transport test_pcl_udp_transport.cpp)
target_link_libraries(test_pcl_udp_transport PRIVATE pcl_core pcl_transport_udp GTest::gtest_main)
if(NOT WIN32)
    target_link_libraries(test_pcl_udp_transport PRIVATE pthread)
endif()
gtest_discover_tests(test_pcl_udp_transport DISCOVERY_MODE PRE_TEST)

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

add_executable(test_pcl_process_runtime test_pcl_process_runtime.cpp)
target_link_libraries(test_pcl_process_runtime PRIVATE pcl_core GTest::gtest_main)
target_compile_definitions(test_pcl_process_runtime PRIVATE
    CAPTURE_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_capture_plugin>"
    SHM_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_shared_memory_plugin>"
    STUB_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_stub_plugin>"
    SEQUENCE_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_sequence_plugin>"
    PORTS_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_ports_plugin>"
    PORTS2_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_ports2_plugin>"
)
add_dependencies(test_pcl_process_runtime
    pcl_transport_capture_plugin
    pcl_transport_shared_memory_plugin
    pcl_codec_stub_plugin
    pcl_codec_sequence_plugin
    pcl_codec_ports_plugin
    pcl_codec_ports2_plugin
)
gtest_discover_tests(test_pcl_process_runtime DISCOVERY_MODE PRE_TEST)

# Template transport: scaffold-specific tests + the reusable conformance
# suite (pcl_transport_conformance.hpp), exercised through an in-memory
# loopback pair.  Other transports can include the same conformance
# header -- see the docstring at the top of pcl_transport_conformance.hpp.
add_executable(test_pcl_template_transport test_pcl_template_transport.cpp)
target_link_libraries(test_pcl_template_transport
    PRIVATE pcl_core pcl_transport_template GTest::gtest_main)
target_include_directories(test_pcl_template_transport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(NOT WIN32)
    target_link_libraries(test_pcl_template_transport PRIVATE pthread)
endif()
gtest_discover_tests(test_pcl_template_transport DISCOVERY_MODE PRE_TEST)

# Threading-model conformance suite (groups A-D): proves executor-thread
# ownership, prompt egress, non-inline callbacks, and shutdown safety across
# the template, shared-memory, UDP, and socket transports.
add_executable(test_pcl_transport_threading test_pcl_transport_threading.cpp)
target_link_libraries(test_pcl_transport_threading
    PRIVATE pcl_core
            pcl_transport_template
            pcl_transport_shared_memory
            pcl_transport_udp
            pcl_transport_socket
            GTest::gtest_main)
target_include_directories(test_pcl_transport_threading PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(NOT WIN32)
    target_link_libraries(test_pcl_transport_threading PRIVATE pthread)
endif()
if(WIN32)
    target_link_libraries(test_pcl_transport_threading PRIVATE ws2_32)
endif()
gtest_discover_tests(test_pcl_transport_threading DISCOVERY_MODE PRE_TEST)

add_executable(test_pcl_apos_transport test_pcl_apos_transport.cpp)
target_link_libraries(test_pcl_apos_transport
    PRIVATE pcl_core pcl_transport_apos GTest::gtest_main)
target_include_directories(test_pcl_apos_transport PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_ROOT_DIR}/bindings/apos)
if(NOT WIN32)
    target_link_libraries(test_pcl_apos_transport PRIVATE pthread)
endif()
gtest_discover_tests(test_pcl_apos_transport DISCOVERY_MODE PRE_TEST)

# Multi-process port-system stress test (worker binary + python driver).
add_subdirectory(stress)
