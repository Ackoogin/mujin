cmake_minimum_required(VERSION 3.21)
project(pcl_coverage LANGUAGES C CXX)

set(CMAKE_C_STANDARD   11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -- Coverage flags ------------------------------------------------------
add_compile_options(--coverage -O0 -g -fprofile-abs-path)
add_link_options(--coverage)

# -- GoogleTest via FetchContent ------------------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
  GIT_SHALLOW    TRUE
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Register the gtest_discover_tests() results with CTest so
# `ctest --test-dir <build>` runs the suite; without this CTest reports
# zero tests even though every binary is discovered.
enable_testing()

# -- PCL core library -----------------------------------------------------
set(WORKSPACE_ROOT "${CMAKE_SOURCE_DIR}/../..")
set(PCL_ROOT "${WORKSPACE_ROOT}/subprojects/PCL")

add_library(pcl_core STATIC
  ${PCL_ROOT}/src/pcl_alloc.c
  ${PCL_ROOT}/src/pcl_capabilities.c
  ${PCL_ROOT}/src/pcl_codec_registry.c
  ${PCL_ROOT}/src/pcl_container.c
  ${PCL_ROOT}/src/pcl_executor.c
  ${PCL_ROOT}/src/pcl_plugin_loader.c
  ${PCL_ROOT}/src/pcl_process_runtime.c
  ${PCL_ROOT}/src/pcl_transport_routing.c
  ${PCL_ROOT}/src/pcl_log.c
  ${PCL_ROOT}/src/pcl_bridge.c
)
target_include_directories(pcl_core PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_core PRIVATE ${PCL_ROOT}/src)
set_target_properties(pcl_core PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
  POSITION_INDEPENDENT_CODE ON
)
find_package(Threads REQUIRED)
target_link_libraries(pcl_core PRIVATE Threads::Threads)
target_link_libraries(pcl_core PUBLIC ${CMAKE_DL_LIBS})

# -- PCL socket transport library ----------------------------------------
add_library(pcl_transport_socket STATIC
  ${PCL_ROOT}/src/pcl_transport_socket.c
)
target_include_directories(pcl_transport_socket PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_transport_socket PRIVATE ${PCL_ROOT}/src)
target_link_libraries(pcl_transport_socket PUBLIC pcl_core)
if(WIN32)
  target_link_libraries(pcl_transport_socket PUBLIC ws2_32)
endif()
set_target_properties(pcl_transport_socket PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# -- PCL UDP transport library -------------------------------------------
add_library(pcl_transport_udp STATIC
  ${PCL_ROOT}/src/pcl_transport_udp.c
)
target_include_directories(pcl_transport_udp PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_transport_udp PRIVATE ${PCL_ROOT}/src)
target_link_libraries(pcl_transport_udp PUBLIC pcl_core Threads::Threads)
if(WIN32)
  target_link_libraries(pcl_transport_udp PUBLIC ws2_32)
endif()
set_target_properties(pcl_transport_udp PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# -- PCL shared-memory transport library ---------------------------------
add_library(pcl_transport_shared_memory STATIC
  ${PCL_ROOT}/src/pcl_transport_shared_memory.c
)
target_include_directories(pcl_transport_shared_memory PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_transport_shared_memory PRIVATE ${PCL_ROOT}/src)
target_link_libraries(pcl_transport_shared_memory PUBLIC pcl_core Threads::Threads)
if(NOT WIN32)
  # rt is required for shm_open/shm_unlink on Linux glibc.
  target_link_libraries(pcl_transport_shared_memory PUBLIC rt)
endif()
set_target_properties(pcl_transport_shared_memory PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# -- PCL template transport library --------------------------------------
add_library(pcl_transport_template STATIC
  ${PCL_ROOT}/src/pcl_transport_template.c
)
target_include_directories(pcl_transport_template PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_transport_template PRIVATE ${PCL_ROOT}/src)
target_link_libraries(pcl_transport_template PUBLIC pcl_core Threads::Threads)
set_target_properties(pcl_transport_template PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# -- APOS stub + APOS transport library ----------------------------------
add_library(apos_stub STATIC
  ${PCL_ROOT}/bindings/apos/apos.c
)
target_include_directories(apos_stub PUBLIC
  ${PCL_ROOT}/bindings/apos
)
target_link_libraries(apos_stub PUBLIC Threads::Threads)
set_target_properties(apos_stub PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

add_library(pcl_transport_apos STATIC
  ${PCL_ROOT}/src/pcl_transport_apos.c
)
target_include_directories(pcl_transport_apos PUBLIC
  ${PCL_ROOT}/include
)
target_include_directories(pcl_transport_apos PRIVATE
  ${PCL_ROOT}/bindings/apos
  ${PCL_ROOT}/src
)
target_link_libraries(pcl_transport_apos
  PUBLIC pcl_core pcl_transport_template apos_stub)
set_target_properties(pcl_transport_apos PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# -- Helper to add a PCL test executable --------------------------------
function(add_pcl_test name src)
  add_executable(${name} ${PCL_ROOT}/tests/${src})
  target_link_libraries(${name} PRIVATE pcl_core GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
endfunction()

add_pcl_test(test_pcl_alloc       test_pcl_alloc.cpp)
add_pcl_test(test_pcl_lifecycle   test_pcl_lifecycle.cpp)
add_pcl_test(test_pcl_executor    test_pcl_executor.cpp)
add_pcl_test(test_pcl_log         test_pcl_log.cpp)
add_pcl_test(test_pcl_robustness  test_pcl_robustness.cpp)
add_pcl_test(test_pcl_streaming   test_pcl_streaming.cpp)
add_pcl_test(test_pcl_bridge      test_pcl_bridge.cpp)
# Bridge test needs internal header (PCL_MAX_PORTS) to trigger port-overflow branch.
target_include_directories(test_pcl_bridge PRIVATE ${PCL_ROOT}/src)
add_pcl_test(test_pcl_dining      test_pcl_dining.cpp)
# Dining test needs internal header to trigger port-overflow branch in bridge.
target_include_directories(test_pcl_dining PRIVATE ${PCL_ROOT}/src)

# Socket transport tests
add_executable(test_pcl_socket_transport ${PCL_ROOT}/tests/test_pcl_socket_transport.cpp)
target_link_libraries(test_pcl_socket_transport PRIVATE pcl_core pcl_transport_socket GTest::gtest_main)
gtest_discover_tests(test_pcl_socket_transport DISCOVERY_MODE PRE_TEST)

if(NOT MSVC)
  add_executable(test_pcl_socket_faults ${PCL_ROOT}/tests/test_pcl_socket_faults.cpp)
  target_link_libraries(test_pcl_socket_faults PRIVATE pcl_core pcl_transport_socket GTest::gtest_main)
  target_include_directories(test_pcl_socket_faults PRIVATE ${PCL_ROOT}/src)
  target_link_options(test_pcl_socket_faults PRIVATE
    -Wl,--wrap=malloc,--wrap=calloc)
  gtest_discover_tests(test_pcl_socket_faults DISCOVERY_MODE PRE_TEST)
endif()

# UDP transport tests
add_executable(test_pcl_udp_transport ${PCL_ROOT}/tests/test_pcl_udp_transport.cpp)
target_link_libraries(test_pcl_udp_transport PRIVATE pcl_core pcl_transport_udp GTest::gtest_main)
gtest_discover_tests(test_pcl_udp_transport DISCOVERY_MODE PRE_TEST)

# Shared-memory transport tests (peer helper + main test process)
add_executable(pcl_shm_peer_helper ${PCL_ROOT}/tests/pcl_shm_peer_helper.cpp)
target_link_libraries(pcl_shm_peer_helper PRIVATE pcl_core pcl_transport_shared_memory)

add_executable(test_pcl_shared_memory_transport
  ${PCL_ROOT}/tests/test_pcl_shared_memory_transport.cpp)
target_link_libraries(test_pcl_shared_memory_transport
  PRIVATE pcl_core pcl_transport_shared_memory GTest::gtest_main)
add_dependencies(test_pcl_shared_memory_transport pcl_shm_peer_helper)
target_compile_definitions(test_pcl_shared_memory_transport PRIVATE
  PCL_SHM_PEER_HELPER_PATH="$<TARGET_FILE:pcl_shm_peer_helper>"
)
gtest_discover_tests(test_pcl_shared_memory_transport DISCOVERY_MODE PRE_TEST)

# Template transport tests (uses internal conformance header)
add_executable(test_pcl_template_transport
  ${PCL_ROOT}/tests/test_pcl_template_transport.cpp)
target_link_libraries(test_pcl_template_transport
  PRIVATE pcl_core pcl_transport_template GTest::gtest_main)
target_include_directories(test_pcl_template_transport PRIVATE
  ${PCL_ROOT}/tests)
gtest_discover_tests(test_pcl_template_transport DISCOVERY_MODE PRE_TEST)

# APOS transport tests
add_executable(test_pcl_apos_transport
  ${PCL_ROOT}/tests/test_pcl_apos_transport.cpp)
target_link_libraries(test_pcl_apos_transport
  PRIVATE pcl_core pcl_transport_apos GTest::gtest_main)
target_include_directories(test_pcl_apos_transport PRIVATE
  ${PCL_ROOT}/tests
  ${PCL_ROOT}/bindings/apos)
gtest_discover_tests(test_pcl_apos_transport DISCOVERY_MODE PRE_TEST)

# Threading-model conformance suite (PCL.075/PCL.076)
add_executable(test_pcl_transport_threading
  ${PCL_ROOT}/tests/test_pcl_transport_threading.cpp)
target_link_libraries(test_pcl_transport_threading
  PRIVATE pcl_core
          pcl_transport_template
          pcl_transport_shared_memory
          pcl_transport_udp
          pcl_transport_socket
          GTest::gtest_main)
target_include_directories(test_pcl_transport_threading PRIVATE ${PCL_ROOT}/tests)
gtest_discover_tests(test_pcl_transport_threading DISCOVERY_MODE PRE_TEST)

# Codec registry tests
add_pcl_test(test_pcl_codec_registry test_pcl_codec_registry.cpp)

# Reference transport plugins + test plugins for loader/capabilities/routing tests
add_library(pcl_transport_socket_plugin MODULE
  ${PCL_ROOT}/src/pcl_transport_socket_plugin.c)
target_link_libraries(pcl_transport_socket_plugin PRIVATE pcl_transport_socket Threads::Threads)

add_library(pcl_transport_udp_plugin MODULE
  ${PCL_ROOT}/src/pcl_transport_udp_plugin.c)
target_link_libraries(pcl_transport_udp_plugin PRIVATE pcl_transport_udp Threads::Threads)

add_library(pcl_transport_shared_memory_plugin MODULE
  ${PCL_ROOT}/src/pcl_transport_shared_memory_plugin.c)
target_link_libraries(pcl_transport_shared_memory_plugin PRIVATE pcl_transport_shared_memory Threads::Threads)

add_library(pcl_transport_capture_plugin MODULE ${PCL_ROOT}/tests/pcl_transport_capture_plugin.c)
target_link_libraries(pcl_transport_capture_plugin PRIVATE pcl_core)

add_library(pcl_codec_stub_plugin MODULE ${PCL_ROOT}/tests/pcl_codec_stub_plugin.c)
target_link_libraries(pcl_codec_stub_plugin PRIVATE pcl_core)

add_library(pcl_codec_badabi_plugin MODULE ${PCL_ROOT}/tests/pcl_codec_badabi_plugin.c)
target_link_libraries(pcl_codec_badabi_plugin PRIVATE pcl_core)

add_library(pcl_transport_nocaps_plugin MODULE ${PCL_ROOT}/tests/pcl_transport_nocaps_plugin.c)
target_link_libraries(pcl_transport_nocaps_plugin PRIVATE pcl_core)

add_library(pcl_transport_wrongabi_plugin MODULE ${PCL_ROOT}/tests/pcl_transport_wrongabi_plugin.c)
target_link_libraries(pcl_transport_wrongabi_plugin PRIVATE pcl_core)

add_library(pcl_transport_noentry_plugin MODULE ${PCL_ROOT}/tests/pcl_transport_noentry_plugin.c)
target_link_libraries(pcl_transport_noentry_plugin PRIVATE pcl_core)

add_library(pcl_codec_sequence_plugin MODULE
  ${PCL_ROOT}/tests/pcl_codec_sequence_plugin.c)
target_link_libraries(pcl_codec_sequence_plugin PRIVATE pcl_core)

# Plugin loader tests
add_executable(test_pcl_plugin_loader ${PCL_ROOT}/tests/test_pcl_plugin_loader.cpp)
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

# Routing manifest tests
add_executable(test_pcl_transport_routing ${PCL_ROOT}/tests/test_pcl_transport_routing.cpp)
target_link_libraries(test_pcl_transport_routing PRIVATE pcl_core GTest::gtest_main)
target_include_directories(test_pcl_transport_routing PRIVATE ${PCL_ROOT}/src)
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

# Capabilities/QoS tests
add_executable(test_pcl_capabilities ${PCL_ROOT}/tests/test_pcl_capabilities.cpp)
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

# Process runtime tests need real transport and codec plugins for the
# deployment-file success paths.
add_executable(test_pcl_process_runtime
  ${PCL_ROOT}/tests/test_pcl_process_runtime.cpp)
target_link_libraries(test_pcl_process_runtime PRIVATE pcl_core GTest::gtest_main)
target_compile_definitions(test_pcl_process_runtime PRIVATE
  CAPTURE_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_capture_plugin>"
  SHM_TRANSPORT_PLUGIN_PATH="$<TARGET_FILE:pcl_transport_shared_memory_plugin>"
  STUB_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_stub_plugin>"
  SEQUENCE_CODEC_PLUGIN_PATH="$<TARGET_FILE:pcl_codec_sequence_plugin>"
)
add_dependencies(test_pcl_process_runtime
  pcl_transport_capture_plugin
  pcl_transport_shared_memory_plugin
  pcl_codec_stub_plugin
  pcl_codec_sequence_plugin
)
gtest_discover_tests(test_pcl_process_runtime DISCOVERY_MODE PRE_TEST)

# C++ wrapper tests
add_executable(test_pcl_cpp_wrappers ${PCL_ROOT}/tests/test_pcl_cpp_wrappers.cpp)
target_link_libraries(test_pcl_cpp_wrappers PRIVATE pcl_core GTest::gtest_main)
gtest_discover_tests(test_pcl_cpp_wrappers DISCOVERY_MODE PRE_TEST)

# OOM test: built against a SEPARATE copy of pcl_core so its gcda counters
# don't accumulate on the shared pcl_core object files.  The --wrap linker
# flag intercepts thousands of GTest-internal malloc/calloc calls which, when
# sharing the same gcda, causes branch-counter overflow (GCC bug #68080) and
# makes gcovr misreport line 428 as uncovered.
add_library(pcl_core_oom STATIC
  ${PCL_ROOT}/src/pcl_alloc.c
  ${PCL_ROOT}/src/pcl_capabilities.c
  ${PCL_ROOT}/src/pcl_codec_registry.c
  ${PCL_ROOT}/src/pcl_container.c
  ${PCL_ROOT}/src/pcl_executor.c
  ${PCL_ROOT}/src/pcl_plugin_loader.c
  ${PCL_ROOT}/src/pcl_process_runtime.c
  ${PCL_ROOT}/src/pcl_transport_routing.c
  ${PCL_ROOT}/src/pcl_log.c
  ${PCL_ROOT}/src/pcl_bridge.c
)
target_include_directories(pcl_core_oom PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_core_oom PRIVATE ${PCL_ROOT}/src)
set_target_properties(pcl_core_oom PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)
target_link_libraries(pcl_core_oom PRIVATE Threads::Threads)
target_link_libraries(pcl_core_oom PUBLIC ${CMAKE_DL_LIBS})

if(NOT MSVC)
  add_executable(test_pcl_oom ${PCL_ROOT}/tests/test_pcl_oom.cpp)
  target_link_libraries(test_pcl_oom PRIVATE pcl_core_oom GTest::gtest_main)
  target_link_options(test_pcl_oom PRIVATE -Wl,--wrap=malloc,--wrap=calloc)
  gtest_discover_tests(test_pcl_oom DISCOVERY_MODE PRE_TEST)
endif()
