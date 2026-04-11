#include "pyramid_components_tactical_objects_services_consumed_grpc_transport.hpp"

#include "pyramid_services_tactical_objects_grpc_dispatch.hpp"

namespace pyramid::services::tactical_objects::consumed::grpc_transport {

namespace grpc_detail = pyramid::services::tactical_objects::grpc_transport::detail;

grpc::Status ObjectEvidenceServiceImpl::ReadDetail(
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
      "object_evidence.read_detail");
}

grpc::Status ObjectSolutionEvidenceServiceImpl::CreateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
    ::pyramid::data_model::base::Identifier* response) {
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

grpc::Status ObjectSolutionEvidenceServiceImpl::ReadRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectEvidenceRequirement>*
        writer) {
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

grpc::Status ObjectSolutionEvidenceServiceImpl::UpdateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
    ::pyramid::data_model::common::Ack* response) {
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

grpc::Status ObjectSolutionEvidenceServiceImpl::DeleteRequirement(
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
      "object_solution_evidence.delete_requirement");
}

grpc::Status ObjectSourceCapabilityServiceImpl::ReadCapability(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::common::Capability>* writer) {
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

ServerHost::ServerHost(ServiceHandler& handler)
    : object_evidence_service_(handler),
      object_solution_evidence_service_(handler),
      object_source_capability_service_(handler) {}

grpc::Server* ServerHost::start(const std::string& listen_address) {
  grpc::ServerBuilder builder;
  builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&object_evidence_service_);
  builder.RegisterService(&object_solution_evidence_service_);
  builder.RegisterService(&object_source_capability_service_);
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

}  // namespace pyramid::services::tactical_objects::consumed::grpc_transport
