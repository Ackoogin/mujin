#include <gtest/gtest.h>

#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;
namespace provided = pyramid::services::tactical_objects::provided;

namespace {

constexpr const char* kSvcCreateRequirement =
    "object_of_interest.create_requirement";

constexpr auto kExpectedIdentifier = "grpc-interest-42";

std::atomic<bool> g_handler_called{false};
std::atomic<int> g_captured_policy{
    static_cast<int>(proto_common::DATA_POLICY_UNSPECIFIED)};
std::thread::id g_handler_thread_id;
std::thread::id g_executor_thread_id;

std::string serialize(const google::protobuf::MessageLite& message) {
  std::string bytes;
  EXPECT_TRUE(message.SerializeToString(&bytes));
  return bytes;
}

pcl_status_t handleCreateRequirement(pcl_container_t*,
                                     const pcl_msg_t* request,
                                     pcl_msg_t* response,
                                     pcl_svc_context_t*,
                                     void*) {
  g_handler_called.store(true);
  g_handler_thread_id = std::this_thread::get_id();

  EXPECT_NE(request, nullptr);
  EXPECT_NE(response, nullptr);
  EXPECT_NE(request->type_name, nullptr);
  EXPECT_STREQ(request->type_name, "application/protobuf");

  proto_tactical::ObjectInterestRequirement decoded;
  EXPECT_TRUE(decoded.ParseFromArray(request->data, static_cast<int>(request->size)));
  g_captured_policy.store(static_cast<int>(decoded.policy()));

  proto_base::Identifier encoded;
  encoded.set_value(kExpectedIdentifier);
  const auto response_bytes = serialize(encoded);

  auto* storage = std::malloc(response_bytes.size());
  EXPECT_NE(storage, nullptr);
  std::memcpy(storage, response_bytes.data(), response_bytes.size());

  response->data = storage;
  response->size = static_cast<uint32_t>(response_bytes.size());
  response->type_name = "application/protobuf";
  return PCL_OK;
}

pcl_status_t onConfigure(pcl_container_t* container, void*) {
  auto* port = pcl_container_add_service(
      container, kSvcCreateRequirement, "application/protobuf",
      handleCreateRequirement, nullptr);
  return port ? PCL_OK : PCL_ERR_NOMEM;
}

}  // namespace

TEST(GrpcTransportSmoke, UnaryCreateRequirementRoundTrip) {
  constexpr auto kAddress = "127.0.0.1:50091";

  g_handler_called.store(false);
  g_captured_policy.store(
      static_cast<int>(proto_common::DATA_POLICY_UNSPECIFIED));
  g_handler_thread_id = {};
  g_executor_thread_id = {};

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = onConfigure;

  pcl_executor_t* executor = pcl_executor_create();
  ASSERT_NE(executor, nullptr);

  pcl_container_t* container =
      pcl_container_create("grpc_smoke_service", &callbacks, nullptr);
  ASSERT_NE(container, nullptr);
  ASSERT_EQ(pcl_container_configure(container), PCL_OK);
  ASSERT_EQ(pcl_container_activate(container), PCL_OK);
  ASSERT_EQ(pcl_executor_add(executor, container), PCL_OK);

  std::atomic<bool> spin_stop{false};
  std::thread executor_thread([&] {
    g_executor_thread_id = std::this_thread::get_id();
    while (!spin_stop.load()) {
      pcl_executor_spin_once(executor, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  auto host = provided::grpc_transport::buildServer(kAddress, executor);
  ASSERT_NE(host, nullptr);
  ASSERT_NE(host->get(), nullptr);

  auto channel =
      grpc::CreateChannel(kAddress, grpc::InsecureChannelCredentials());
  auto stub = proto_services::Object_Of_Interest_Service::NewStub(channel);
  ASSERT_NE(stub, nullptr);

  proto_tactical::ObjectInterestRequirement request;
  request.set_policy(proto_common::DATA_POLICY_OBTAIN);
  proto_base::Identifier response;
  grpc::ClientContext context;

  const auto status = stub->CreateRequirement(&context, request, &response);

  EXPECT_TRUE(status.ok()) << status.error_message();
  EXPECT_TRUE(g_handler_called.load());
  EXPECT_EQ(g_captured_policy.load(),
            static_cast<int>(proto_common::DATA_POLICY_OBTAIN));
  EXPECT_NE(g_handler_thread_id, std::thread::id{});
  EXPECT_NE(g_executor_thread_id, std::thread::id{});
  EXPECT_NE(g_handler_thread_id, std::this_thread::get_id());
  EXPECT_EQ(g_handler_thread_id, g_executor_thread_id);
  EXPECT_EQ(response.value(), kExpectedIdentifier);

  host->shutdown();
  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}
