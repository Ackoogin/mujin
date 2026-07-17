#include <pcl/pcl_process_runtime.h>

#include <gtest/gtest.h>

namespace {

pcl_status_t stopOnTick(
    pcl_container_t*,
    double,
    void* user_data) {
  pcl_process_runtime_request_shutdown(
      static_cast<pcl_process_runtime_t*>(user_data));
  return PCL_OK;
}

}  // namespace

TEST(PclProcessRuntime, CreatesAndDestroysExecutor) {
  pcl_process_runtime_t* runtime = nullptr;
  ASSERT_EQ(pcl_process_runtime_create(0, &runtime), PCL_OK);
  ASSERT_NE(runtime, nullptr);
  EXPECT_NE(pcl_process_runtime_executor(runtime), nullptr);
  pcl_process_runtime_destroy(runtime);
}

TEST(PclProcessRuntime, RunsComponentUntilShutdownRequest) {
  pcl_process_runtime_t* runtime = nullptr;
  ASSERT_EQ(pcl_process_runtime_create(0, &runtime), PCL_OK);

  pcl_callbacks_t callbacks = {};
  callbacks.on_tick = stopOnTick;
  pcl_container_t* component =
      pcl_container_create("runtime_test", &callbacks, runtime);
  ASSERT_NE(component, nullptr);

  EXPECT_EQ(pcl_process_runtime_run(runtime, component), PCL_OK)
      << pcl_process_runtime_error(runtime);
  EXPECT_EQ(pcl_container_state(component), PCL_STATE_UNCONFIGURED);

  pcl_container_destroy(component);
  pcl_process_runtime_destroy(runtime);
}

TEST(PclProcessRuntime, RejectsMissingPortsFile) {
  pcl_process_runtime_t* runtime = nullptr;
  ASSERT_EQ(pcl_process_runtime_create(0, &runtime), PCL_OK);
  const pcl_process_endpoint_descriptor_t rpc[] = {
      {"fixture.create", PCL_ENDPOINT_CONSUMED},
  };
  const pcl_process_endpoint_descriptor_t pubsub[] = {
      {"fixture.request", PCL_ENDPOINT_PUBLISHER},
  };
  const pcl_process_port_descriptor_t ports[] = {
      {"fixture_request", rpc, 1, pubsub, 1},
  };

  EXPECT_EQ(
      pcl_process_runtime_load_ports_file(
          runtime, "/path/that/does/not/exist.ports", ports, 1),
      PCL_ERR_NOT_FOUND);
  EXPECT_NE(
      std::string(pcl_process_runtime_error(runtime)).find("cannot open"),
      std::string::npos);
  pcl_process_runtime_destroy(runtime);
}
