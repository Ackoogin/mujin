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
  pcl_bridge.c
)

target_include_directories(pcl_core PUBLIC
  ${PCL_ROOT_DIR}/include
)

target_include_directories(pcl_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Force C compilation (not C++)
set_target_properties(pcl_core PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# Platform threading library (needed for sleep/timing)
find_package(Threads REQUIRED)
target_link_libraries(pcl_core PRIVATE Threads::Threads)

# ---------------------------------------------------------------------------
# pcl_transport_socket — TCP socket transport (optional, platform sockets)
# ---------------------------------------------------------------------------
add_library(pcl_transport_socket STATIC
  pcl_transport_socket.c
)

target_include_directories(pcl_transport_socket PUBLIC
  ${PCL_ROOT_DIR}/include
)

target_link_libraries(pcl_transport_socket PUBLIC pcl_core)

if(WIN32)
  target_link_libraries(pcl_transport_socket PRIVATE ws2_32)
endif()

set_target_properties(pcl_transport_socket PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# ---------------------------------------------------------------------------
# pcl_transport_udp — UDP datagram transport (pub/sub only, no RPC)
# ---------------------------------------------------------------------------
add_library(pcl_transport_udp STATIC
  pcl_transport_udp.c
)

target_include_directories(pcl_transport_udp PUBLIC
  ${PCL_ROOT_DIR}/include
)

target_link_libraries(pcl_transport_udp PUBLIC pcl_core)

if(WIN32)
  target_link_libraries(pcl_transport_udp PRIVATE ws2_32)
endif()

set_target_properties(pcl_transport_udp PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# ---------------------------------------------------------------------------
# pcl_transport_shared_memory — central shared-memory bus transport
# ---------------------------------------------------------------------------
add_library(pcl_transport_shared_memory STATIC
  pcl_transport_shared_memory.c
)

target_include_directories(pcl_transport_shared_memory PUBLIC
  ${PCL_ROOT_DIR}/include
)

target_include_directories(pcl_transport_shared_memory PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(pcl_transport_shared_memory PUBLIC pcl_core)

set_target_properties(pcl_transport_shared_memory PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# Install
install(TARGETS pcl_core pcl_transport_socket pcl_transport_udp pcl_transport_shared_memory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY ${PCL_ROOT_DIR}/include/pcl/
  DESTINATION include/pcl
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)
