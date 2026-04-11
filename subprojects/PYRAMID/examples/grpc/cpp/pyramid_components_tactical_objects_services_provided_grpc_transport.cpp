#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"

#include "pyramid_services_tactical_objects_grpc_dispatch.hpp"

namespace pyramid::services::tactical_objects::provided::grpc_transport {

namespace grpc_detail = pyramid::services::tactical_objects::grpc_transport::detail;

grpc::Status MatchingObjectsServiceImpl::ReadMatch(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer) {
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

grpc::Status ObjectOfInterestServiceImpl::CreateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
    ::pyramid::data_model::base::Identifier* response) {
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

grpc::Status ObjectOfInterestServiceImpl::ReadRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectInterestRequirement>*
        writer) {
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

grpc::Status ObjectOfInterestServiceImpl::UpdateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
    ::pyramid::data_model::common::Ack* response) {
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

grpc::Status ObjectOfInterestServiceImpl::DeleteRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response) {
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

grpc::Status SpecificObjectDetailServiceImpl::ReadDetail(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer) {
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

ServerHost::ServerHost(ServiceHandler& handler)
    : matching_objects_service_(handler),
      object_of_interest_service_(handler),
      specific_object_detail_service_(handler) {}

grpc::Server* ServerHost::start(const std::string& listen_address) {
  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&matching_objects_service_);
  builder.RegisterService(&object_of_interest_service_);
  builder.RegisterService(&specific_object_detail_service_);
  server_ = builder.BuildAndStart();
  return server_.get();
}

void ServerHost::shutdown() {
  if (server_) {
    server_->Shutdown();
  }
}

void ServerHost::wait() {
  if (server_) {
    server_->Wait();
  }
}

std::unique_ptr<ServerHost> buildServer(const std::string& listen_address,
                                        ServiceHandler& handler) {
  auto host = std::make_unique<ServerHost>(handler);
  host->start(listen_address);
  return host;
}

}  // namespace pyramid::services::tactical_objects::provided::grpc_transport
