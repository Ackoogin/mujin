// Auto-generated gRPC transport — do not edit

#include "pyramid_components_tactical_objects_services_consumed_grpc_transport.hpp"

namespace pyramid::components::tactical_objects::services::consumed::grpc_transport {

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const ObjectEvidenceRequirement& /*request*/) {
    return {};
}

std::vector<ObjectEvidenceRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const ObjectEvidenceRequirement& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return {};
}

std::vector<Capability>
ServiceHandler::handleReadCapability(const Query& /*request*/) {
    return {};
}

grpc::Status Object_Evidence_ServiceImpl::ReadDetail(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectDetail>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadDetail(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Object_Solution_Evidence_ServiceImpl::CreateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
    ::pyramid::data_model::base::Identifier* response)
{
    *response = handler_.handleCreateRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Object_Solution_Evidence_ServiceImpl::ReadRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectEvidenceRequirement>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadRequirement(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Object_Solution_Evidence_ServiceImpl::UpdateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectEvidenceRequirement* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleUpdateRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Object_Solution_Evidence_ServiceImpl::DeleteRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleDeleteRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Object_Source_Capability_ServiceImpl::ReadCapability(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::common::Capability>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadCapability(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

std::unique_ptr<grpc::Server> buildServer(
    const std::string& listen_address,
    ServiceHandler& handler)
{
    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());

    auto object_evidence_service = std::make_unique<Object_Evidence_ServiceImpl>(handler);
    builder.RegisterService(object_evidence_service.get());

    auto object_solution_evidence_service = std::make_unique<Object_Solution_Evidence_ServiceImpl>(handler);
    builder.RegisterService(object_solution_evidence_service.get());

    auto object_source_capability_service = std::make_unique<Object_Source_Capability_ServiceImpl>(handler);
    builder.RegisterService(object_source_capability_service.get());

    return builder.BuildAndStart();
}

} // namespace pyramid::components::tactical_objects::services::consumed::grpc_transport
