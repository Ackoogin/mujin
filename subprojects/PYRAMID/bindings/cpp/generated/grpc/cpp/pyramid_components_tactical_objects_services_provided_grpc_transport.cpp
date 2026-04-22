// Auto-generated gRPC transport — do not edit

#include "pyramid_components_tactical_objects_services_provided_grpc_transport.hpp"

namespace pyramid::components::tactical_objects::services::provided::grpc_transport {

std::vector<ObjectMatch>
ServiceHandler::handleReadMatch(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const ObjectInterestRequirement& /*request*/) {
    return {};
}

std::vector<ObjectInterestRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const ObjectInterestRequirement& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return {};
}

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

grpc::Status Matching_Objects_ServiceImpl::ReadMatch(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectMatch>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadMatch(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Object_Of_Interest_ServiceImpl::CreateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
    ::pyramid::data_model::base::Identifier* response)
{
    *response = handler_.handleCreateRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Object_Of_Interest_ServiceImpl::ReadRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::tactical::ObjectInterestRequirement>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadRequirement(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Object_Of_Interest_ServiceImpl::UpdateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::tactical::ObjectInterestRequirement* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleUpdateRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Object_Of_Interest_ServiceImpl::DeleteRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleDeleteRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Specific_Object_Detail_ServiceImpl::ReadDetail(
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

std::unique_ptr<grpc::Server> buildServer(
    const std::string& listen_address,
    ServiceHandler& handler)
{
    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());

    auto matching_objects_service = std::make_unique<Matching_Objects_ServiceImpl>(handler);
    builder.RegisterService(matching_objects_service.get());

    auto object_of_interest_service = std::make_unique<Object_Of_Interest_ServiceImpl>(handler);
    builder.RegisterService(object_of_interest_service.get());

    auto specific_object_detail_service = std::make_unique<Specific_Object_Detail_ServiceImpl>(handler);
    builder.RegisterService(specific_object_detail_service.get());

    return builder.BuildAndStart();
}

} // namespace pyramid::components::tactical_objects::services::provided::grpc_transport
