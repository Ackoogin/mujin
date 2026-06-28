/// \file test_grpc_coupled_plugin_e2e.cpp
/// \brief Live end-to-end test for the coupled gRPC target plugin.
///
/// Loads libpyramid_grpc_coupled_plugin.so through the generic PCL plugin
/// loader, configures it as a server for the tactical_objects/provided
/// component, then drives a real unary RPC from a separate gRPC client stub.
/// Asserts the PCL service handler runs on the executor thread (proving the
/// plugin-started server routes ingress to the executor) and that the response
/// decodes correctly. This is the plugin-loaded analogue of
/// test_grpc_transport_smoke, which exercises the static buildGrpcServer path.

#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport.h>
}

#include "pyramid/components/pyramid.components.tactical_objects.services.provided.grpc.pb.h"
#include "pyramid/data_model/pyramid.data_model.base.pb.h"
#include "pyramid/data_model/pyramid.data_model.common.pb.h"
#include "pyramid/data_model/pyramid.data_model.tactical.pb.h"
#include "pyramid_codec_plugin_test_paths.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;
namespace proto_services = pyramid::components::tactical_objects::services::provided;

namespace {

constexpr const char* kSvcCreateRequirement =
    "object_of_interest.create_requirement";
constexpr const char* kExpectedIdentifier = "grpc-plugin-interest-7";

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

proto_tactical::ObjectInterestRequirement makeCreateRequirementRequest() {
  proto_tactical::ObjectInterestRequirement request;
  auto* base = request.mutable_base()->mutable_base();
  base->mutable_id()->set_value("grpc-plugin-interest-7");
  base->mutable_source()->set_value("grpc-plugin-client");
  request.set_source(proto_tactical::OBJECT_SOURCE_RADAR);
  request.set_policy(proto_common::DATA_POLICY_OBTAIN);
  request.add_dimension(proto_common::BATTLE_DIMENSION_GROUND);
  return request;
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
  EXPECT_TRUE(
      decoded.ParseFromArray(request->data, static_cast<int>(request->size)));
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

uint16_t pickLoopbackPort() {
#if defined(_WIN32)
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 0;
  SOCKET tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp == INVALID_SOCKET) {
    WSACleanup();
    return 0;
  }
#else
  int tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp < 0) return 0;
#endif
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  uint16_t port = 0;
  if (bind(tmp, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
#if defined(_WIN32)
    int len = static_cast<int>(sizeof(addr));
#else
    socklen_t len = sizeof(addr);
#endif
    if (getsockname(tmp, reinterpret_cast<sockaddr*>(&addr), &len) == 0) {
      port = ntohs(addr.sin_port);
    }
  }
#if defined(_WIN32)
  closesocket(tmp);
  WSACleanup();
#else
  close(tmp);
#endif
  return port;
}

std::string buildConfig(pcl_executor_t* executor, const std::string& address) {
  const auto exec_value = static_cast<uint64_t>(
      reinterpret_cast<uintptr_t>(executor));
  return std::string("{\"executor\":") + std::to_string(exec_value) +
         ",\"component\":\"tactical_objects\",\"role\":\"provided\""
         ",\"address\":\"" +
         address + "\"}";
}

std::string buildClientConfig(pcl_executor_t* executor,
                              const std::string& address) {
  const auto exec_value = static_cast<uint64_t>(
      reinterpret_cast<uintptr_t>(executor));
  return std::string("{\"executor\":") + std::to_string(exec_value) +
         ",\"mode\":\"client\",\"address\":\"" + address + "\"}";
}

struct CapturedResponse {
  bool called = false;
  std::string bytes;
};

void captureResponse(const pcl_msg_t* resp, void* user_data) {
  auto* captured = static_cast<CapturedResponse*>(user_data);
  captured->called = true;
  if (resp && resp->data && resp->size > 0U) {
    const auto* bytes = static_cast<const char*>(resp->data);
    captured->bytes.assign(bytes, bytes + resp->size);
  }
}

}  // namespace

TEST(GrpcCoupledPluginE2E, UnaryRoundTripThroughLoadedPlugin) {
  ASSERT_NE(kPyramidGrpcCoupledPlugin, nullptr);

  const auto port = pickLoopbackPort();
  ASSERT_NE(port, 0);
  const auto address = std::string("127.0.0.1:") + std::to_string(port);

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
      pcl_container_create("grpc_plugin_service", &callbacks, nullptr);
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

  // Load the coupled plugin and start its gRPC server for tactical_objects.
  const auto config = buildConfig(executor, address);
  pcl_plugin_handle_t* handle = nullptr;
  const pcl_transport_t* transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(kPyramidGrpcCoupledPlugin, config.c_str(),
                                      &handle, &transport),
            PCL_OK);
  ASSERT_NE(handle, nullptr);
  ASSERT_NE(transport, nullptr);

  // Drive a real unary RPC from a separate gRPC client against the plugin.
  auto channel =
      grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
  auto stub = proto_services::Object_Of_Interest_Service::NewStub(channel);
  ASSERT_NE(stub, nullptr);

  const auto request = makeCreateRequirementRequest();
  proto_base::Identifier response;
  grpc::ClientContext context;
  const auto status = stub->CreateRequirement(&context, request, &response);

  EXPECT_TRUE(status.ok()) << status.error_message();
  EXPECT_TRUE(g_handler_called.load());
  EXPECT_EQ(g_captured_policy.load(),
            static_cast<int>(proto_common::DATA_POLICY_OBTAIN));
  EXPECT_NE(g_handler_thread_id, std::thread::id{});
  EXPECT_NE(g_handler_thread_id, std::this_thread::get_id());
  EXPECT_EQ(g_handler_thread_id, g_executor_thread_id);
  EXPECT_EQ(response.value(), kExpectedIdentifier);

  // Tear down the plugin transport via its private destroy symbol, then unload.
  auto* destroy = reinterpret_cast<void (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(handle, "pcl_grpc_transport_plugin_destroy"));
  ASSERT_NE(destroy, nullptr);
  destroy(transport);
  pcl_plugin_unload(handle);

  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}

// Proves the consumed (client) direction: a client-mode plugin invokes a unary
// service through the generated typed stub against a server-mode plugin, and the
// PCL handler behind the server runs and returns a decodable response. Together
// with the test above this exercises PCL <-> plugin <-> plugin <-> PCL both ways.
TEST(GrpcCoupledPluginE2E, ConsumedUnaryThroughLoadedPlugin) {
  ASSERT_NE(kPyramidGrpcCoupledPlugin, nullptr);

  const auto port = pickLoopbackPort();
  ASSERT_NE(port, 0);
  const auto address = std::string("127.0.0.1:") + std::to_string(port);

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
      pcl_container_create("grpc_plugin_service", &callbacks, nullptr);
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

  // Server-mode plugin: hosts tactical_objects/provided.
  const auto server_config = buildConfig(executor, address);
  pcl_plugin_handle_t* server_handle = nullptr;
  const pcl_transport_t* server_transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(kPyramidGrpcCoupledPlugin,
                                      server_config.c_str(), &server_handle,
                                      &server_transport),
            PCL_OK);

  // Client-mode plugin: dials the server and consumes through invoke_async.
  const auto client_config = buildClientConfig(executor, address);
  pcl_plugin_handle_t* client_handle = nullptr;
  const pcl_transport_t* client_transport = nullptr;
  ASSERT_EQ(pcl_plugin_load_transport(kPyramidGrpcCoupledPlugin,
                                      client_config.c_str(), &client_handle,
                                      &client_transport),
            PCL_OK);
  ASSERT_NE(client_transport, nullptr);
  ASSERT_NE(client_transport->invoke_async, nullptr);

  const auto request = makeCreateRequirementRequest();
  std::string request_bytes;
  ASSERT_TRUE(request.SerializeToString(&request_bytes));

  pcl_msg_t request_msg{};
  request_msg.data = request_bytes.data();
  request_msg.size = static_cast<uint32_t>(request_bytes.size());
  request_msg.type_name = "application/protobuf";

  CapturedResponse captured;
  const auto rc = client_transport->invoke_async(
      client_transport->adapter_ctx, "object_of_interest.create_requirement",
      &request_msg, captureResponse, &captured);

  EXPECT_EQ(rc, PCL_OK);
  EXPECT_TRUE(captured.called);
  EXPECT_TRUE(g_handler_called.load());

  proto_base::Identifier decoded;
  ASSERT_TRUE(decoded.ParseFromArray(
      captured.bytes.data(), static_cast<int>(captured.bytes.size())));
  EXPECT_EQ(decoded.value(), kExpectedIdentifier);

  auto* destroy = reinterpret_cast<void (*)(const pcl_transport_t*)>(
      pcl_plugin_symbol(server_handle, "pcl_grpc_transport_plugin_destroy"));
  ASSERT_NE(destroy, nullptr);
  destroy(client_transport);
  destroy(server_transport);
  pcl_plugin_unload(client_handle);
  pcl_plugin_unload(server_handle);

  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}
