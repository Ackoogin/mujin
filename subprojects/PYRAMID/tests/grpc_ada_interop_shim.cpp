#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"
#include "pyramid_services_tactical_objects_grpc_dispatch_api.hpp"

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

namespace proto_base = pyramid::data_model::base;
namespace proto_common = pyramid::data_model::common;
namespace proto_services = pyramid::components::tactical_objects::services::provided;
namespace proto_tactical = pyramid::data_model::tactical;
namespace provided = pyramid::services::tactical_objects::provided;

namespace pyramid::services::tactical_objects::provided {

class ServiceHandler {};

void dispatch(ServiceHandler&,
              ServiceChannel channel,
              const void* request_buf,
              std::size_t request_size,
              const char* content_type,
              void** response_buf,
              std::size_t* response_size);

}  // namespace pyramid::services::tactical_objects::provided

namespace {

constexpr std::size_t kResponseBufferSize = 256;
constexpr const char* kExpectedIdentifier = "ada-grpc-interest-42";

std::mutex g_mutex;
provided::ServiceHandler g_handler;
std::unique_ptr<provided::grpc_transport::ServerHost> g_server;

std::string serialize(const google::protobuf::MessageLite& message) {
  std::string bytes;
  if (!message.SerializeToString(&bytes)) {
    return {};
  }
  return bytes;
}

void setResponseString(char* response, const std::string& text) {
  if (!response) {
    return;
  }
  std::snprintf(response, kResponseBufferSize, "%s", text.c_str());
}

proto_common::DataPolicy parsePolicy(const char* text) {
  if (!text) {
    return proto_common::DATA_POLICY_UNSPECIFIED;
  }
  if (std::strcmp(text, "DATA_POLICY_OBTAIN") == 0) {
    return proto_common::DATA_POLICY_OBTAIN;
  }
  if (std::strcmp(text, "DATA_POLICY_QUERY") == 0) {
    return proto_common::DATA_POLICY_QUERY;
  }
  return proto_common::DATA_POLICY_UNSPECIFIED;
}

}  // namespace

namespace pyramid::services::tactical_objects::provided {

void dispatch(ServiceHandler&,
              ServiceChannel channel,
              const void* request_buf,
              std::size_t request_size,
              const char* content_type,
              void** response_buf,
              std::size_t* response_size) {
  if (!response_buf || !response_size ||
      channel != ServiceChannel::CreateRequirement ||
      !content_type ||
      std::strcmp(content_type, "application/protobuf") != 0) {
    *response_buf = nullptr;
    *response_size = 0;
    return;
  }

  proto_tactical::ObjectInterestRequirement request;
  if (!request.ParseFromArray(request_buf, static_cast<int>(request_size))) {
    *response_buf = nullptr;
    *response_size = 0;
    return;
  }

  proto_base::Identifier response;
  response.set_value(request.policy() == proto_common::DATA_POLICY_OBTAIN
                         ? kExpectedIdentifier
                         : "unexpected-policy");

  const auto bytes = serialize(response);
  *response_size = bytes.size();
  if (bytes.empty()) {
    *response_buf = nullptr;
    *response_size = 0;
    return;
  }

  auto* storage = std::malloc(bytes.size());
  if (!storage) {
    *response_buf = nullptr;
    *response_size = 0;
    return;
  }
  std::memcpy(storage, bytes.data(), bytes.size());
  *response_buf = storage;
}

}  // namespace pyramid::services::tactical_objects::provided

extern "C" {

__declspec(dllexport) void pyramid_grpc_server_start(const char* address) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_server) {
    g_server->shutdown();
    g_server.reset();
  }
  g_server = provided::grpc_transport::buildServer(
      address ? std::string(address) : std::string("127.0.0.1:50101"), g_handler);
}

__declspec(dllexport) void pyramid_grpc_server_stop() {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_server) {
    g_server->shutdown();
    g_server.reset();
  }
}

__declspec(dllexport) void grpc_object_of_interest_service_create_requirement(
    void* channel, void* request, void* response) {
  auto* endpoint = static_cast<const char*>(channel);
  auto* policy_name = static_cast<const char*>(request);
  auto* response_buffer = static_cast<char*>(response);

  if (!endpoint || !response_buffer) {
    setResponseString(response_buffer, "ERROR:null-argument");
    return;
  }

  auto grpc_channel =
      grpc::CreateChannel(endpoint, grpc::InsecureChannelCredentials());
  auto stub = proto_services::Object_Of_Interest_Service::NewStub(grpc_channel);
  if (!stub) {
    setResponseString(response_buffer, "ERROR:create-stub");
    return;
  }

  proto_tactical::ObjectInterestRequirement grpc_request;
  grpc_request.set_policy(parsePolicy(policy_name));

  proto_base::Identifier grpc_response;
  grpc::ClientContext context;
  const auto status =
      stub->CreateRequirement(&context, grpc_request, &grpc_response);
  if (!status.ok()) {
    setResponseString(response_buffer,
                      std::string("ERROR:") + status.error_message());
    return;
  }

  setResponseString(response_buffer, grpc_response.value());
}

}  // extern "C"
