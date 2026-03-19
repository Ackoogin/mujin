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

# -- PCL core library (the three C files we want to cover) ----------------
set(PCL_ROOT "${CMAKE_SOURCE_DIR}/../..")   # /home/user/ame

add_library(pcl_core STATIC
  ${PCL_ROOT}/src/pcl/pcl_container.c
  ${PCL_ROOT}/src/pcl/pcl_executor.c
  ${PCL_ROOT}/src/pcl/pcl_log.c
  ${PCL_ROOT}/src/pcl/pcl_bridge.c
)
target_include_directories(pcl_core PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_core PRIVATE ${PCL_ROOT}/src/pcl)
set_target_properties(pcl_core PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)
find_package(Threads REQUIRED)
target_link_libraries(pcl_core PRIVATE Threads::Threads)

# -- Helper to add a PCL test executable --------------------------------
function(add_pcl_test name src)
  add_executable(${name} ${PCL_ROOT}/tests/${src})
  target_link_libraries(${name} PRIVATE pcl_core GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
endfunction()

add_pcl_test(test_pcl_lifecycle   test_pcl_lifecycle.cpp)
add_pcl_test(test_pcl_executor    test_pcl_executor.cpp)
add_pcl_test(test_pcl_log         test_pcl_log.cpp)
add_pcl_test(test_pcl_robustness  test_pcl_robustness.cpp)
add_pcl_test(test_pcl_dining      test_pcl_dining.cpp)
# Dining test needs internal header to trigger port-overflow branch in bridge.
target_include_directories(test_pcl_dining PRIVATE ${PCL_ROOT}/src/pcl)

# OOM test: built against a SEPARATE copy of pcl_core so its gcda counters
# don't accumulate on the shared pcl_core object files.  The --wrap linker
# flag intercepts thousands of GTest-internal malloc/calloc calls which, when
# sharing the same gcda, causes branch-counter overflow (GCC bug #68080) and
# makes gcovr misreport line 428 as uncovered.
add_library(pcl_core_oom STATIC
  ${PCL_ROOT}/src/pcl/pcl_container.c
  ${PCL_ROOT}/src/pcl/pcl_executor.c
  ${PCL_ROOT}/src/pcl/pcl_log.c
  ${PCL_ROOT}/src/pcl/pcl_bridge.c
)
target_include_directories(pcl_core_oom PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_core_oom PRIVATE ${PCL_ROOT}/src/pcl)
set_target_properties(pcl_core_oom PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)
target_link_libraries(pcl_core_oom PRIVATE Threads::Threads)

if(NOT MSVC)
  add_executable(test_pcl_oom ${PCL_ROOT}/tests/test_pcl_oom.cpp)
  target_link_libraries(test_pcl_oom PRIVATE pcl_core_oom GTest::gtest_main)
  target_link_options(test_pcl_oom PRIVATE -Wl,--wrap=malloc,--wrap=calloc)
  gtest_discover_tests(test_pcl_oom DISCOVERY_MODE PRE_TEST)
endif()
