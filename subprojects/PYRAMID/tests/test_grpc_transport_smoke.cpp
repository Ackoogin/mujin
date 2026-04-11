#include <gtest/gtest.h>

#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"
#include "pyramid_services_tactical_objects_grpc_dispatch_api.hpp"

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_tactical = pyramid::data_model::tactical;
namespace provided = pyramid::services::tactical_objects::provided;

namespace {

constexpr auto kExpectedIdentifier = "grpc-interest-42";

bool g_dispatch_called = false;
provided::ServiceChannel g_last_channel = provided::ServiceChannel::ReadMatch;
std::string g_last_content_type;
proto_common::DataPolicy g_captured_policy =
    proto_common::DATA_POLICY_UNSPECIFIED;

std::string serialize(const google::protobuf::MessageLite& message) {
  std::string bytes;
  EXPECT_TRUE(message.SerializeToString(&bytes));
  return bytes;
}

void writeResponseBytes(const std::string& bytes, void** response_buf,
                        size_t* response_size) {
  *response_size = bytes.size();
  if (bytes.empty()) {
    *response_buf = nullptr;
    return;
  }
  auto* storage = std::malloc(bytes.size());
  ASSERT_NE(storage, nullptr);
  std::memcpy(storage, bytes.data(), bytes.size());
  *response_buf = storage;
}

}  // namespace

namespace pyramid::services::tactical_objects::provided {

class ServiceHandler {};

void dispatch(ServiceHandler&,
              ServiceChannel channel,
              const void* request_buf,
              size_t request_size,
              const char* content_type,
              void** response_buf,
              size_t* response_size) {
  g_dispatch_called = true;
  g_last_channel = channel;
  g_last_content_type = content_type ? content_type : "";

  EXPECT_EQ(channel, ServiceChannel::CreateRequirement);
  EXPECT_STREQ(content_type, "application/protobuf");

  proto_tactical::ObjectInterestRequirement request;
  ASSERT_TRUE(request.ParseFromArray(request_buf, static_cast<int>(request_size)));
  g_captured_policy = request.policy();

  proto_base::Identifier response;
  response.set_value(kExpectedIdentifier);
  writeResponseBytes(serialize(response), response_buf, response_size);
}

}  // namespace pyramid::services::tactical_objects::provided

namespace {

TEST(GrpcTransportSmoke, UnaryCreateRequirementRoundTrip) {
  constexpr auto kAddress = "127.0.0.1:50091";

  g_dispatch_called = false;
  g_last_channel = provided::ServiceChannel::ReadMatch;
  g_last_content_type.clear();
  g_captured_policy = proto_common::DATA_POLICY_UNSPECIFIED;

  provided::ServiceHandler handler;
  auto host = provided::grpc_transport::buildServer(kAddress, handler);
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
  EXPECT_TRUE(g_dispatch_called);
  EXPECT_EQ(g_last_channel, provided::ServiceChannel::CreateRequirement);
  EXPECT_EQ(g_last_content_type, "application/protobuf");
  EXPECT_EQ(g_captured_policy, proto_common::DATA_POLICY_OBTAIN);
  EXPECT_EQ(response.value(), kExpectedIdentifier);

  host->shutdown();
}

}  // namespace
