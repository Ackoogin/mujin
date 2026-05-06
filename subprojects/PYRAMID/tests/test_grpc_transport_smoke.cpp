#include <gtest/gtest.h>

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"
#include "pyramid_data_model_types.hpp"
#include "pyramid_services_tactical_objects_protobuf_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <cstring>
#include <memory>
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

namespace pyramid::components::tactical_objects::services::provided {

class GrpcServer {
public:
  GrpcServer();
  GrpcServer(GrpcServer&&) noexcept;
  GrpcServer& operator=(GrpcServer&&) noexcept;
  GrpcServer(const GrpcServer&) = delete;
  GrpcServer& operator=(const GrpcServer&) = delete;
  ~GrpcServer();

  bool started() const;
  explicit operator bool() const { return started(); }
  void shutdown();

private:
  struct Impl;
  explicit GrpcServer(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend GrpcServer buildGrpcServer(const std::string& listen_address,
                                    pcl_executor_t* executor);
};

GrpcServer buildGrpcServer(const std::string& listen_address,
                           pcl_executor_t* executor);

}  // namespace pyramid::components::tactical_objects::services::provided

namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;
namespace domain = pyramid::domain_model;
namespace protobuf_codec = pyramid::services::tactical_objects::protobuf_codec;
namespace provided = pyramid::components::tactical_objects::services::provided;

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

proto_tactical::ObjectInterestRequirement makeCreateRequirementRequest() {
  proto_tactical::ObjectInterestRequirement request;
  auto* base = request.mutable_base()->mutable_base();
  base->mutable_id()->set_value("grpc-interest-42");
  base->mutable_source()->set_value("grpc-smoke-client");
  request.set_source(proto_tactical::OBJECT_SOURCE_RADAR);
  request.set_policy(proto_common::DATA_POLICY_OBTAIN);
  request.add_dimension(proto_common::BATTLE_DIMENSION_GROUND);
  request.add_dimension(proto_common::BATTLE_DIMENSION_AIR);
  auto* point = request.mutable_point()->mutable_position();
  point->mutable_latitude()->set_radians(51.477811);
  point->mutable_longitude()->set_radians(-0.001475);
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

TEST(GrpcTransportSmoke, DomainAndWireTypesCoexist) {
  domain::ObjectDetail domain_value;
  domain_value.id = "coexist-1";
  domain_value.identity = domain::StandardIdentity::Friendly;
  domain_value.dimension = domain::BattleDimension::Air;
  domain_value.position.latitude = 0.5;
  domain_value.position.longitude = -1.25;

  const auto bytes = protobuf_codec::toBinary(domain_value);
  const auto decoded = protobuf_codec::fromBinaryObjectDetail(bytes);

  proto_tactical::ObjectDetail wire_value;
  ASSERT_TRUE(wire_value.ParseFromString(bytes));
  ASSERT_TRUE(wire_value.has_base());
  ASSERT_TRUE(wire_value.base().has_id());

  EXPECT_EQ(decoded.id, "coexist-1");
  EXPECT_EQ(wire_value.base().id().value(), "coexist-1");
  EXPECT_EQ(wire_value.identity(), proto_common::STANDARD_IDENTITY_FRIENDLY);
}

uint16_t pickLoopbackPort() {
#if defined(_WIN32)
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    return 0;
  }
  SOCKET tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp == INVALID_SOCKET) {
    WSACleanup();
    return 0;
  }
#else
  int tmp = socket(AF_INET, SOCK_STREAM, 0);
  if (tmp < 0) {
    return 0;
  }
#endif

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;

  if (bind(tmp, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
#if defined(_WIN32)
    closesocket(tmp);
    WSACleanup();
#else
    close(tmp);
#endif
    return 0;
  }

#if defined(_WIN32)
  int len = static_cast<int>(sizeof(addr));
#else
  socklen_t len = sizeof(addr);
#endif
  if (getsockname(tmp, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
#if defined(_WIN32)
    closesocket(tmp);
    WSACleanup();
#else
    close(tmp);
#endif
    return 0;
  }

  const auto port = ntohs(addr.sin_port);
#if defined(_WIN32)
  closesocket(tmp);
  WSACleanup();
#else
  close(tmp);
#endif
  return port;
}

TEST(GrpcTransportSmoke, UnaryCreateRequirementRoundTrip) {
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

  auto host = provided::buildGrpcServer(address, executor);
  ASSERT_TRUE(host.started());

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
  EXPECT_NE(g_executor_thread_id, std::thread::id{});
  EXPECT_NE(g_handler_thread_id, std::this_thread::get_id());
  EXPECT_EQ(g_handler_thread_id, g_executor_thread_id);
  EXPECT_EQ(response.value(), kExpectedIdentifier);

  host.shutdown();
  spin_stop.store(true);
  executor_thread.join();
  pcl_container_destroy(container);
  pcl_executor_destroy(executor);
}
