# ---------------------------------------------------------------------------
# PYRAMID port-system stress test
# ---------------------------------------------------------------------------
# pcl_port_stress_worker is one process of a multi-process stress run.  It is
# a plain executable (not a CTest test): the run_port_stress.py driver in
# this directory launches many workers against the runtime-loaded transport
# plugins and aggregates their results.  The worker links only pcl_core --
# every transport comes in through pcl_plugin_load_transport, the same path a
# deployed component uses.
add_executable(pcl_port_stress_worker pcl_port_stress_worker.cpp)
target_link_libraries(pcl_port_stress_worker PRIVATE pcl_core)
if(NOT WIN32)
    target_link_libraries(pcl_port_stress_worker PRIVATE pthread)
endif()
add_dependencies(pcl_port_stress_worker
    pcl_transport_shared_memory_plugin
    pcl_transport_socket_plugin
    pcl_transport_udp_plugin
)
