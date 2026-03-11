cmake_minimum_required(VERSION 3.21)
project(pcl_coverage LANGUAGES C CXX)

set(CMAKE_C_STANDARD   17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Coverage flags ──────────────────────────────────────────────────────
add_compile_options(--coverage -O0 -g -fprofile-abs-path)
add_link_options(--coverage)

# ── GoogleTest via FetchContent ──────────────────────────────────────────
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

# ── PCL core library (the three C files we want to cover) ────────────────
set(PCL_ROOT "${CMAKE_SOURCE_DIR}/../..")   # /home/user/mujin

add_library(pcl_core STATIC
  ${PCL_ROOT}/src/pcl/pcl_container.c
  ${PCL_ROOT}/src/pcl/pcl_executor.c
  ${PCL_ROOT}/src/pcl/pcl_log.c
)
target_include_directories(pcl_core PUBLIC  ${PCL_ROOT}/include)
target_include_directories(pcl_core PRIVATE ${PCL_ROOT}/src/pcl)
set_target_properties(pcl_core PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 17
)
find_package(Threads REQUIRED)
target_link_libraries(pcl_core PRIVATE Threads::Threads)

# ── Helper to add a PCL test executable ────────────────────────────────
function(add_pcl_test name src)
  add_executable(${name} ${PCL_ROOT}/tests/${src})
  target_link_libraries(${name} PRIVATE pcl_core GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
endfunction()

add_pcl_test(test_pcl_lifecycle   test_pcl_lifecycle.cpp)
add_pcl_test(test_pcl_executor    test_pcl_executor.cpp)
add_pcl_test(test_pcl_log         test_pcl_log.cpp)
add_pcl_test(test_pcl_robustness  test_pcl_robustness.cpp)

# OOM test: links with --wrap=malloc,calloc to inject allocation failures.
add_executable(test_pcl_oom ${PCL_ROOT}/tests/test_pcl_oom.cpp)
target_link_libraries(test_pcl_oom PRIVATE pcl_core GTest::gtest_main)
target_link_options(test_pcl_oom PRIVATE -Wl,--wrap=malloc,--wrap=calloc)
gtest_discover_tests(test_pcl_oom DISCOVERY_MODE PRE_TEST)
