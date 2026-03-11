# ---------------------------------------------------------------------------
# pcl_core — PYRAMID Container Library (pure C, zero dependencies)
# ---------------------------------------------------------------------------
# Compiled as C (not C++) to guarantee ABI stability across compilers.
# C++ header-only wrappers are in include/pcl/*.hpp.
# ---------------------------------------------------------------------------

add_library(pcl_core STATIC
  pcl_container.c
  pcl_executor.c
  pcl_log.c
)

target_include_directories(pcl_core PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_include_directories(pcl_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Force C compilation (not C++)
set_target_properties(pcl_core PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 17
  C_STANDARD_REQUIRED ON
)

# Platform threading library (needed for sleep/timing)
find_package(Threads REQUIRED)
target_link_libraries(pcl_core PRIVATE Threads::Threads)

# Install
install(TARGETS pcl_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/pcl/
  DESTINATION include/pcl
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)
