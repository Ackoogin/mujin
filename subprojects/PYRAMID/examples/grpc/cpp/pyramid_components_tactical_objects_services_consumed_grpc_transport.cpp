#include "pyramid_components_tactical_objects_services_consumed_grpc_transport.hpp"

#include "pyramid_services_tactical_objects_grpc_dispatch.hpp"
#include "pyramid_services_tactical_objects_grpc_dispatch_api.hpp"

#include "pyramid/components/tactical_objects/services/consumed.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <utility>

namespace pyramid::services::tactical_objects::consumed::grpc_transport {

namespace grpc_detail = pyramid::services::tactical_objects::grpc_transport::detail;

class ObjectEvidenceServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Evidence_Service::Service {
 public:
  explicit ObjectEvidenceServiceImpl(ServiceHandler& handler) : handler_(handler) {}

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
          dispatch(handler_, ServiceChannel::ReadDetail, request_buf, request_size,
                   grpc_detail::kProtobufContentType, response_buf, response_size);
        },
        "object_evidence.read_detail");
  }

 private:
  ServiceHandler& handler_;
};

class ObjectSolutionEvidenceServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Solution_Evidence_Service::Service {
 public:
  explicit ObjectSolutionEvidenceServiceImpl(ServiceHandler& handler)
      : handler_(handler) {}

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
          dispatch(handler_, ServiceChannel::CreateRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
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
          dispatch(handler_, ServiceChannel::ReadRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
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
          dispatch(handler_, ServiceChannel::UpdateRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
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
          dispatch(handler_, ServiceChannel::DeleteRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
        },
        "object_solution_evidence.delete_requirement");
  }

 private:
  ServiceHandler& handler_;
};

class ObjectSourceCapabilityServiceImpl final
    : public ::pyramid::components::tactical_objects::services::consumed::
          Object_Source_Capability_Service::Service {
 public:
  explicit ObjectSourceCapabilityServiceImpl(ServiceHandler& handler)
      : handler_(handler) {}

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
          dispatch(handler_, ServiceChannel::ReadCapability, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
        },
        "object_source_capability.read_capability");
  }

 private:
  ServiceHandler& handler_;
};

class ServerHost::Impl {
 public:
  explicit Impl(ServiceHandler& handler)
      : object_evidence_service(handler),
        object_solution_evidence_service(handler),
        object_source_capability_service(handler) {}

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

ServerHost::ServerHost(ServiceHandler& handler) : impl_(std::make_unique<Impl>(handler)) {}

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
                                        ServiceHandler& handler) {
  auto host = std::make_unique<ServerHost>(handler);
  host->start(listen_address);
  return host;
}

}  // namespace pyramid::services::tactical_objects::consumed::grpc_transport
