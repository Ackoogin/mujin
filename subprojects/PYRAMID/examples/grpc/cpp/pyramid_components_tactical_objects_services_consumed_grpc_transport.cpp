#include "pyramid_components_tactical_objects_services_consumed_grpc_transport.hpp"

#include "pyramid_services_tactical_objects_grpc_dispatch.hpp"

#include "pyramid/components/tactical_objects/services/consumed.grpc.pb.h"

#include <pcl/pcl_executor.h>

#include <grpcpp/grpcpp.h>

#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pyramid::services::tactical_objects::consumed::grpc_transport {

namespace grpc_detail = pyramid::services::tactical_objects::grpc_transport::detail;

namespace {

constexpr const char* kSvcReadDetail = "object_evidence.read_detail";
constexpr const char* kSvcCreateRequirement =
    "object_solution_evidence.create_requirement";
constexpr const char* kSvcReadRequirement =
    "object_solution_evidence.read_requirement";
constexpr const char* kSvcUpdateRequirement =
    "object_solution_evidence.update_requirement";
constexpr const char* kSvcDeleteRequirement =
    "object_solution_evidence.delete_requirement";
constexpr const char* kSvcReadCapability =
    "object_source_capability.read_capability";

struct UnaryState {
  std::promise<std::string> promise;
};

void unaryResponseCallback(const pcl_msg_t* response, void* user_data) {
  auto* state = static_cast<UnaryState*>(user_data);
  if (!state) {
    return;
  }
  if (!response || !response->data || response->size == 0) {
    state->promise.set_value(std::string{});
    return;
  }
  state->promise.set_value(
      std::string(static_cast<const char*>(response->data), response->size));
}

std::pair<grpc_detail::UniqueResponseBuffer, size_t> invokeUnaryService(
    pcl_executor_t* executor, const char* service_name,
    const std::string& request_bytes) {
  UnaryState state;
  pcl_msg_t request{};
  request.data = request_bytes.data();
  request.size = static_cast<uint32_t>(request_bytes.size());
  request.type_name = grpc_detail::kProtobufContentType;

  const auto post_rc = pcl_executor_post_service_request(
      executor, service_name, &request, unaryResponseCallback, &state);
  if (post_rc != PCL_OK) {
    throw std::runtime_error("failed to post service request to executor");
  }

  const auto response = state.promise.get_future().get();
  void* storage = nullptr;
  if (!response.empty()) {
    storage = std::malloc(response.size());
    if (!storage) {
      throw std::runtime_error("failed to allocate response buffer");
    }
    std::memcpy(storage, response.data(), response.size());
  }
  return {grpc_detail::UniqueResponseBuffer(storage), response.size()};
}

}  // namespace

class ObjectEvidenceServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Evidence_Service::Service {
 public:
  explicit ObjectEvidenceServiceImpl(pcl_executor_t* executor)
      : executor_(executor) {}

  grpc::Status ReadDetail(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer)
      override {
    return grpc_detail::dispatchServerStream<
        ::pyramid::data_model::common::Query,
        ::pyramid::data_model::tactical::ObjectDetail>(
        request, writer,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          auto result = invokeUnaryService(
              executor_, kSvcReadDetail,
              std::string(static_cast<const char*>(request_buf), request_size));
          *response_buf = result.first.release();
          *response_size = result.second;
        },
        "object_evidence.read_detail");
  }

 private:
  pcl_executor_t* executor_;
};

class ObjectSolutionEvidenceServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Solution_Evidence_Service::Service {
 public:
  explicit ObjectSolutionEvidenceServiceImpl(pcl_executor_t* executor)
      : executor_(executor) {}

  grpc::Status CreateRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
      ::pyramid::data_model::base::Identifier* response) override {
    return grpc_detail::dispatchUnary<
        ::pyramid::data_model::tactical::ObjectEvidenceRequirement,
        ::pyramid::data_model::base::Identifier>(
        request, response,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          auto result = invokeUnaryService(
              executor_, kSvcCreateRequirement,
              std::string(static_cast<const char*>(request_buf), request_size));
          *response_buf = result.first.release();
          *response_size = result.second;
        },
        "object_solution_evidence.create_requirement");
  }

  grpc::Status ReadRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectEvidenceRequirement>*
          writer) override {
    return grpc_detail::dispatchServerStream<
        ::pyramid::data_model::common::Query,
        ::pyramid::data_model::tactical::ObjectEvidenceRequirement>(
        request, writer,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          auto result = invokeUnaryService(
              executor_, kSvcReadRequirement,
              std::string(static_cast<const char*>(request_buf), request_size));
          *response_buf = result.first.release();
          *response_size = result.second;
        },
        "object_solution_evidence.read_requirement");
  }

  grpc::Status UpdateRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
      ::pyramid::data_model::common::Ack* response) override {
    return grpc_detail::dispatchUnary<
        ::pyramid::data_model::tactical::ObjectEvidenceRequirement,
        ::pyramid::data_model::common::Ack>(
        request, response,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          auto result = invokeUnaryService(
              executor_, kSvcUpdateRequirement,
              std::string(static_cast<const char*>(request_buf), request_size));
          *response_buf = result.first.release();
          *response_size = result.second;
        },
        "object_solution_evidence.update_requirement");
  }

  grpc::Status DeleteRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::base::Identifier* request,
      ::pyramid::data_model::common::Ack* response) override {
    return grpc_detail::dispatchUnary<::pyramid::data_model::base::Identifier,
                                      ::pyramid::data_model::common::Ack>(
        request, response,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          auto result = invokeUnaryService(
              executor_, kSvcDeleteRequirement,
              std::string(static_cast<const char*>(request_buf), request_size));
          *response_buf = result.first.release();
          *response_size = result.second;
        },
        "object_solution_evidence.delete_requirement");
  }

 private:
  pcl_executor_t* executor_;
};

class ObjectSourceCapabilityServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Source_Capability_Service::Service {
 public:
  explicit ObjectSourceCapabilityServiceImpl(pcl_executor_t* executor)
      : executor_(executor) {}

  grpc::Status ReadCapability(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::common::Capability>* writer)
      override {
    return grpc_detail::dispatchServerStream<
        ::pyramid::data_model::common::Query,
        ::pyramid::data_model::common::Capability>(
        request, writer,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          auto result = invokeUnaryService(
              executor_, kSvcReadCapability,
              std::string(static_cast<const char*>(request_buf), request_size));
          *response_buf = result.first.release();
          *response_size = result.second;
        },
        "object_source_capability.read_capability");
  }

 private:
  pcl_executor_t* executor_;
};

class ServerHost::Impl {
 public:
  explicit Impl(pcl_executor_t* executor)
      : object_evidence_service(executor),
        object_solution_evidence_service(executor),
        object_source_capability_service(executor) {}

  grpc::Server* start(const std::string& listen_address) {
    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&object_evidence_service);
    builder.RegisterService(&object_solution_evidence_service);
    builder.RegisterService(&object_source_capability_service);
    server = builder.BuildAndStart();
    return server.get();
  }

  void shutdown() {
    if (server) {
      server->Shutdown();
    }
  }

  void wait() {
    if (server) {
      server->Wait();
    }
  }

  grpc::Server* get() const { return server.get(); }

  ObjectEvidenceServiceImpl object_evidence_service;
  ObjectSolutionEvidenceServiceImpl object_solution_evidence_service;
  ObjectSourceCapabilityServiceImpl object_source_capability_service;
  std::unique_ptr<grpc::Server> server;
};

ServerHost::ServerHost(pcl_executor_t* executor)
    : impl_(std::make_unique<Impl>(executor)) {}

ServerHost::~ServerHost() = default;

grpc::Server* ServerHost::start(const std::string& listen_address) {
  return impl_->start(listen_address);
}

void ServerHost::shutdown() {
  impl_->shutdown();
}

void ServerHost::wait() {
  impl_->wait();
}

grpc::Server* ServerHost::get() const {
  return impl_->get();
}

std::unique_ptr<ServerHost> buildServer(const std::string& listen_address,
                                        pcl_executor_t* executor) {
  auto host = std::make_unique<ServerHost>(executor);
  host->start(listen_address);
  return host;
}

}  // namespace pyramid::services::tactical_objects::consumed::grpc_transport
