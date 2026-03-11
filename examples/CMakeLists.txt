# ---------------------------------------------------------------------------
# PCL examples
# ---------------------------------------------------------------------------
add_executable(wm_container_example wm_container_example.c)
target_link_libraries(wm_container_example PRIVATE pcl_core)

add_executable(external_io_bridge_example external_io_bridge_example.c)
target_link_libraries(external_io_bridge_example PRIVATE pcl_core)
