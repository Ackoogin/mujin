// Auto-generated gRPC transport — do not edit

#include "pyramid_components_autonomy_backend_services_provided_grpc_transport.hpp"

namespace pyramid::components::autonomy_backend::services::provided::grpc_transport {

std::vector<Capabilities>
ServiceHandler::handleReadCapabilities(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const PlanningExecutionRequirement& /*request*/) {
    return {};
}

std::vector<PlanningExecutionRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const PlanningExecutionRequirement& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateState(const StateUpdate& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateState(const StateUpdate& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleDeleteState(const Identifier& /*request*/) {
    return {};
}

std::vector<Plan>
ServiceHandler::handleReadPlan(const Query& /*request*/) {
    return {};
}

std::vector<ExecutionRun>
ServiceHandler::handleReadRun(const Query& /*request*/) {
    return {};
}

std::vector<RequirementPlacement>
ServiceHandler::handleReadPlacement(const Query& /*request*/) {
    return {};
}

grpc::Status Capabilities_ServiceImpl::ReadCapabilities(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::Capabilities>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadCapabilities(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Planning_Execution_ServiceImpl::CreateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::PlanningExecutionRequirement* request,
    ::pyramid::data_model::base::Identifier* response)
{
    *response = handler_.handleCreateRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Planning_Execution_ServiceImpl::ReadRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::PlanningExecutionRequirement>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadRequirement(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Planning_Execution_ServiceImpl::UpdateRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::PlanningExecutionRequirement* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleUpdateRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status Planning_Execution_ServiceImpl::DeleteRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleDeleteRequirement(*request);
    return grpc::Status::OK;
}

grpc::Status State_ServiceImpl::CreateState(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::StateUpdate* request,
    ::pyramid::data_model::base::Identifier* response)
{
    *response = handler_.handleCreateState(*request);
    return grpc::Status::OK;
}

grpc::Status State_ServiceImpl::UpdateState(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::StateUpdate* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleUpdateState(*request);
    return grpc::Status::OK;
}

grpc::Status State_ServiceImpl::DeleteState(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    *response = handler_.handleDeleteState(*request);
    return grpc::Status::OK;
}

grpc::Status Plan_ServiceImpl::ReadPlan(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::Plan>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadPlan(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Execution_Run_ServiceImpl::ReadRun(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::ExecutionRun>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadRun(*request);
    for (const auto& item : results) {
        writer->Write(item);
    }
    return grpc::Status::OK;
}

grpc::Status Requirement_Placement_ServiceImpl::ReadPlacement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::RequirementPlacement>* writer)
{
    // Delegate to handler, write each result to stream
    auto results = handler_.handleReadPlacement(*request);
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

    auto capabilities_service = std::make_unique<Capabilities_ServiceImpl>(handler);
    builder.RegisterService(capabilities_service.get());

    auto planning_execution_service = std::make_unique<Planning_Execution_ServiceImpl>(handler);
    builder.RegisterService(planning_execution_service.get());

    auto state_service = std::make_unique<State_ServiceImpl>(handler);
    builder.RegisterService(state_service.get());

    auto plan_service = std::make_unique<Plan_ServiceImpl>(handler);
    builder.RegisterService(plan_service.get());

    auto execution_run_service = std::make_unique<Execution_Run_ServiceImpl>(handler);
    builder.RegisterService(execution_run_service.get());

    auto requirement_placement_service = std::make_unique<Requirement_Placement_ServiceImpl>(handler);
    builder.RegisterService(requirement_placement_service.get());

    return builder.BuildAndStart();
}

} // namespace pyramid::components::autonomy_backend::services::provided::grpc_transport
