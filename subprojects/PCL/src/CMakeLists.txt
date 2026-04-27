# ---------------------------------------------------------------------------
# pcl_core -- PYRAMID Container Library (pure C, zero dependencies)
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
# pcl_transport_socket -- TCP socket transport (optional, platform sockets)
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
# pcl_transport_udp -- UDP datagram transport (pub/sub only, no RPC)
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
# pcl_transport_shared_memory -- central shared-memory bus transport
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

# ---------------------------------------------------------------------------
# pcl_transport_template -- engineer-extendable transport scaffold
# ---------------------------------------------------------------------------
# Provides the standard PCL threading model (dedicated send + recv worker
# threads) so the engineer only needs to supply blocking I/O hooks.  See
# include/pcl/pcl_transport_template.h for the integration contract.
# ---------------------------------------------------------------------------
add_library(pcl_transport_template STATIC
  pcl_transport_template.c
)

target_include_directories(pcl_transport_template PUBLIC
  ${PCL_ROOT_DIR}/include
)

target_link_libraries(pcl_transport_template PUBLIC pcl_core Threads::Threads)

set_target_properties(pcl_transport_template PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# ---------------------------------------------------------------------------
# apos_stub + pcl_transport_apos -- APOS Local Virtual Channel transport
# ---------------------------------------------------------------------------
add_library(apos_stub STATIC
  ${PCL_ROOT_DIR}/bindings/apos/apos.c
)

target_include_directories(apos_stub PUBLIC
  ${PCL_ROOT_DIR}/bindings/apos
)

target_link_libraries(apos_stub PUBLIC Threads::Threads)

set_target_properties(apos_stub PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

add_library(pcl_transport_apos STATIC
  pcl_transport_apos.c
)

target_include_directories(pcl_transport_apos PUBLIC
  ${PCL_ROOT_DIR}/include
)

target_include_directories(pcl_transport_apos PRIVATE
  ${PCL_ROOT_DIR}/bindings/apos
)

target_link_libraries(pcl_transport_apos
  PUBLIC pcl_core pcl_transport_template apos_stub)

set_target_properties(pcl_transport_apos PROPERTIES
  LINKER_LANGUAGE C
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF
)

# Install
install(TARGETS pcl_core pcl_transport_socket pcl_transport_udp pcl_transport_shared_memory pcl_transport_template pcl_transport_apos apos_stub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY ${PCL_ROOT_DIR}/include/pcl/
  DESTINATION include/pcl
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)
