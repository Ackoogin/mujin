#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"

#include "pyramid_services_tactical_objects_grpc_dispatch.hpp"
#include "pyramid_services_tactical_objects_grpc_dispatch_api.hpp"

#include "pyramid/components/tactical_objects/services/provided.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <utility>

namespace pyramid::services::tactical_objects::provided::grpc_transport {

namespace grpc_detail = pyramid::services::tactical_objects::grpc_transport::detail;

class MatchingObjectsServiceImpl final
    : public ::pyramid::components::tactical_objects::services::provided::
          Matching_Objects_Service::Service {
 public:
  explicit MatchingObjectsServiceImpl(ServiceHandler& handler) : handler_(handler) {}

  grpc::Status ReadMatch(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer)
      override {
    return grpc_detail::dispatchServerStream<
        ::pyramid::data_model::common::Query,
        ::pyramid::data_model::tactical::ObjectMatch>(
        request, writer,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          dispatch(handler_, ServiceChannel::ReadMatch, request_buf, request_size,
                   grpc_detail::kProtobufContentType, response_buf, response_size);
        },
        "matching_objects.read_match");
  }

 private:
  ServiceHandler& handler_;
};

class ObjectOfInterestServiceImpl final
    : public ::pyramid::components::tactical_objects::services::provided::
          Object_Of_Interest_Service::Service {
 public:
  explicit ObjectOfInterestServiceImpl(ServiceHandler& handler) : handler_(handler) {}

  grpc::Status CreateRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
      ::pyramid::data_model::base::Identifier* response) override {
    return grpc_detail::dispatchUnary<
        ::pyramid::data_model::tactical::ObjectInterestRequirement,
        ::pyramid::data_model::base::Identifier>(
        request, response,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          dispatch(handler_, ServiceChannel::CreateRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
        },
        "object_of_interest.create_requirement");
  }

  grpc::Status ReadRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::common::Query* request,
      grpc::ServerWriter<::pyramid::data_model::tactical::ObjectInterestRequirement>*
          writer) override {
    return grpc_detail::dispatchServerStream<
        ::pyramid::data_model::common::Query,
        ::pyramid::data_model::tactical::ObjectInterestRequirement>(
        request, writer,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          dispatch(handler_, ServiceChannel::ReadRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
        },
        "object_of_interest.read_requirement");
  }

  grpc::Status UpdateRequirement(
      grpc::ServerContext* /*context*/,
      const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
      ::pyramid::data_model::common::Ack* response) override {
    return grpc_detail::dispatchUnary<
        ::pyramid::data_model::tactical::ObjectInterestRequirement,
        ::pyramid::data_model::common::Ack>(
        request, response,
        [&](const void* request_buf, size_t request_size, void** response_buf,
            size_t* response_size) {
          dispatch(handler_, ServiceChannel::UpdateRequirement, request_buf,
                   request_size, grpc_detail::kProtobufContentType, response_buf,
                   response_size);
        },
        "object_of_interest.update_requirement");
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
        "object_of_interest.delete_requirement");
  }

 private:
  ServiceHandler& handler_;
};

class SpecificObjectDetailServiceImpl final
    : public ::pyramid::components::tactical_objects::services::provided::
          Specific_Object_Detail_Service::Service {
 public:
  explicit SpecificObjectDetailServiceImpl(ServiceHandler& handler)
      : handler_(handler) {}

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
        "specific_object_detail.read_detail");
  }

 private:
  ServiceHandler& handler_;
};

class ServerHost::Impl {
 public:
  explicit Impl(ServiceHandler& handler)
      : matching_objects_service(handler),
        object_of_interest_service(handler),
        specific_object_detail_service(handler) {}

  grpc::Server* start(const std::string& listen_address) {
    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&matching_objects_service);
    builder.RegisterService(&object_of_interest_service);
    builder.RegisterService(&specific_object_detail_service);
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

  MatchingObjectsServiceImpl matching_objects_service;
  ObjectOfInterestServiceImpl object_of_interest_service;
  SpecificObjectDetailServiceImpl specific_object_detail_service;
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

}  // namespace pyramid::services::tactical_objects::provided::grpc_transport
