// Auto-generated gRPC transport -- do not edit

#include "pyramid_components_autonomy_backend_services_provided_grpc_transport.hpp"

#include <google/protobuf/message_lite.h>
#include <pcl/pcl_types.h>

#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr const char* kProtobufContentType = "application/protobuf";

struct ExecutorResponse {
    pcl_status_t status = PCL_OK;
    std::string content_type;
    std::string payload;
};

struct UnaryState {
    std::promise<ExecutorResponse> promise;
};

void executor_response_callback(const pcl_msg_t* response, void* user_data)
{
    auto* state = static_cast<UnaryState*>(user_data);
    if (!state) {
        return;
    }
    ExecutorResponse out;
    if (response && response->type_name) {
        out.content_type = response->type_name;
    }
    if (response && response->data && response->size > 0U) {
        const auto* bytes = static_cast<const char*>(response->data);
        out.payload.assign(bytes, bytes + response->size);
    }
    state->promise.set_value(std::move(out));
}

ExecutorResponse invoke_executor(pcl_executor_t* executor,
                                 const char* service_name,
                                 const std::string& request_payload)
{
    ExecutorResponse error;
    if (!executor || !service_name || !service_name[0]) {
        error.status = PCL_ERR_INVALID;
        return error;
    }
    UnaryState state;
    pcl_msg_t request{};
    request.data = request_payload.empty() ? nullptr : request_payload.data();
    request.size = static_cast<uint32_t>(request_payload.size());
    request.type_name = kProtobufContentType;
    const auto rc = pcl_executor_post_service_request(
        executor, service_name, &request, executor_response_callback, &state);
    if (rc != PCL_OK) {
        error.status = rc;
        return error;
    }
    return state.promise.get_future().get();
}

std::string serialize_grpc_request(const google::protobuf::MessageLite& request)
{
    std::string payload;
    if (!request.SerializeToString(&payload)) {
        throw std::runtime_error("could not serialize gRPC request");
    }
    return payload;
}

template <typename ResponsePb>
grpc::Status parse_unary_response(const ExecutorResponse& response,
                                  ResponsePb* out,
                                  const char* rpc_name)
{
    if (response.status != PCL_OK) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "executor service dispatch failed");
    }
    if (!out || !out->ParseFromArray(response.payload.data(), static_cast<int>(response.payload.size()))) {
        return grpc::Status(grpc::StatusCode::INTERNAL, std::string("could not parse response for ") + rpc_name);
    }
    return grpc::Status::OK;
}

bool read_varint32(const char*& cursor, const char* end, uint32_t& value)
{
    value = 0U;
    int shift = 0;
    while (cursor < end && shift <= 28) {
        const auto byte = static_cast<unsigned char>(*cursor++);
        value |= static_cast<uint32_t>(byte & 0x7FU) << shift;
        if ((byte & 0x80U) == 0U) {
            return true;
        }
        shift += 7;
    }
    return false;
}

template <typename ResponsePb, typename WriterT>
grpc::Status write_stream_response(const ExecutorResponse& response,
                                  WriterT* writer,
                                  const char* rpc_name)
{
    if (response.status != PCL_OK) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "executor stream dispatch failed");
    }
    if (!writer) {
        return grpc::Status(grpc::StatusCode::INTERNAL, "missing gRPC stream writer");
    }
    const char* cursor = response.payload.data();
    const char* end = cursor + response.payload.size();
    while (cursor < end) {
        uint32_t frame_size = 0U;
        if (!read_varint32(cursor, end, frame_size) || static_cast<size_t>(end - cursor) < frame_size) {
            return grpc::Status(grpc::StatusCode::INTERNAL, std::string("invalid stream frame for ") + rpc_name);
        }
        ResponsePb item;
        if (!item.ParseFromArray(cursor, static_cast<int>(frame_size))) {
            return grpc::Status(grpc::StatusCode::INTERNAL, std::string("could not parse stream frame for ") + rpc_name);
        }
        cursor += frame_size;
        if (!writer->Write(item)) {
            return grpc::Status(grpc::StatusCode::CANCELLED, "client cancelled gRPC stream");
        }
    }
    return grpc::Status::OK;
}

} // namespace

namespace pyramid::components::autonomy_backend::services::provided::grpc_transport {

grpc::Status Capabilities_ServiceImpl::ReadCapabilities(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::Capabilities>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "capabilities.read_capabilities", payload);
        return write_stream_response<::pyramid::data_model::autonomy::Capabilities>(
            executor_response, writer, "ReadCapabilities");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Planning_Requirement_ServiceImpl::CreatePlanningRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::PlanningRequirement* request,
    ::pyramid::data_model::base::Identifier* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "planning_requirement.create_planning_requirement", payload);
        return parse_unary_response(
            executor_response, response, "CreatePlanningRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Planning_Requirement_ServiceImpl::ReadPlanningRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::PlanningRequirement>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "planning_requirement.read_planning_requirement", payload);
        return write_stream_response<::pyramid::data_model::autonomy::PlanningRequirement>(
            executor_response, writer, "ReadPlanningRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Planning_Requirement_ServiceImpl::UpdatePlanningRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::PlanningRequirement* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "planning_requirement.update_planning_requirement", payload);
        return parse_unary_response(
            executor_response, response, "UpdatePlanningRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Planning_Requirement_ServiceImpl::DeletePlanningRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "planning_requirement.delete_planning_requirement", payload);
        return parse_unary_response(
            executor_response, response, "DeletePlanningRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Execution_Requirement_ServiceImpl::CreateExecutionRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::ExecutionRequirement* request,
    ::pyramid::data_model::base::Identifier* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "execution_requirement.create_execution_requirement", payload);
        return parse_unary_response(
            executor_response, response, "CreateExecutionRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Execution_Requirement_ServiceImpl::ReadExecutionRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::ExecutionRequirement>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "execution_requirement.read_execution_requirement", payload);
        return write_stream_response<::pyramid::data_model::autonomy::ExecutionRequirement>(
            executor_response, writer, "ReadExecutionRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Execution_Requirement_ServiceImpl::UpdateExecutionRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::ExecutionRequirement* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "execution_requirement.update_execution_requirement", payload);
        return parse_unary_response(
            executor_response, response, "UpdateExecutionRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Execution_Requirement_ServiceImpl::DeleteExecutionRequirement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "execution_requirement.delete_execution_requirement", payload);
        return parse_unary_response(
            executor_response, response, "DeleteExecutionRequirement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status State_ServiceImpl::CreateState(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::StateUpdate* request,
    ::pyramid::data_model::base::Identifier* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "state.create_state", payload);
        return parse_unary_response(
            executor_response, response, "CreateState");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status State_ServiceImpl::UpdateState(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::StateUpdate* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "state.update_state", payload);
        return parse_unary_response(
            executor_response, response, "UpdateState");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status State_ServiceImpl::DeleteState(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "state.delete_state", payload);
        return parse_unary_response(
            executor_response, response, "DeleteState");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Plan_ServiceImpl::CreatePlan(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::Plan* request,
    ::pyramid::data_model::base::Identifier* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "plan.create_plan", payload);
        return parse_unary_response(
            executor_response, response, "CreatePlan");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Plan_ServiceImpl::ReadPlan(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::Plan>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "plan.read_plan", payload);
        return write_stream_response<::pyramid::data_model::autonomy::Plan>(
            executor_response, writer, "ReadPlan");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Plan_ServiceImpl::UpdatePlan(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::autonomy::Plan* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "plan.update_plan", payload);
        return parse_unary_response(
            executor_response, response, "UpdatePlan");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Plan_ServiceImpl::DeletePlan(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::base::Identifier* request,
    ::pyramid::data_model::common::Ack* response)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "plan.delete_plan", payload);
        return parse_unary_response(
            executor_response, response, "DeletePlan");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Execution_Run_ServiceImpl::ReadRun(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::ExecutionRun>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "execution_run.read_run", payload);
        return write_stream_response<::pyramid::data_model::autonomy::ExecutionRun>(
            executor_response, writer, "ReadRun");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

grpc::Status Requirement_Placement_ServiceImpl::ReadPlacement(
    grpc::ServerContext* /*context*/,
    const ::pyramid::data_model::common::Query* request,
    grpc::ServerWriter<::pyramid::data_model::autonomy::RequirementPlacement>* writer)
{
    try {
        if (!request) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "missing gRPC request");
        }
        const auto payload = serialize_grpc_request(*request);
        const auto executor_response = invoke_executor(
            executor_, "requirement_placement.read_placement", payload);
        return write_stream_response<::pyramid::data_model::autonomy::RequirementPlacement>(
            executor_response, writer, "ReadPlacement");
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

} // namespace pyramid::components::autonomy_backend::services::provided::grpc_transport

namespace pyramid::components::autonomy_backend::services::provided {

class GrpcServer {
public:
    GrpcServer();
    GrpcServer(GrpcServer&&) noexcept;
    GrpcServer& operator=(GrpcServer&&) noexcept;
    GrpcServer(const GrpcServer&) = delete;
    GrpcServer& operator=(const GrpcServer&) = delete;
    ~GrpcServer();

    bool started() const;
    explicit operator bool() const { return started(); }
    void wait();
    void shutdown();

private:
    struct Impl;
    explicit GrpcServer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
    friend GrpcServer buildGrpcServer(const std::string& listen_address,
                                      pcl_executor_t* executor);
};

struct GrpcServer::Impl {
    std::unique_ptr<grpc_transport::Capabilities_ServiceImpl> capabilities_service;
    std::unique_ptr<grpc_transport::Planning_Requirement_ServiceImpl> planning_requirement_service;
    std::unique_ptr<grpc_transport::Execution_Requirement_ServiceImpl> execution_requirement_service;
    std::unique_ptr<grpc_transport::State_ServiceImpl> state_service;
    std::unique_ptr<grpc_transport::Plan_ServiceImpl> plan_service;
    std::unique_ptr<grpc_transport::Execution_Run_ServiceImpl> execution_run_service;
    std::unique_ptr<grpc_transport::Requirement_Placement_ServiceImpl> requirement_placement_service;
    std::unique_ptr<grpc::Server> server;
};

GrpcServer::GrpcServer() = default;

GrpcServer::GrpcServer(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl))
{}

GrpcServer::GrpcServer(GrpcServer&&) noexcept = default;

GrpcServer& GrpcServer::operator=(GrpcServer&&) noexcept = default;

GrpcServer::~GrpcServer() = default;

bool GrpcServer::started() const
{
    return impl_ && impl_->server != nullptr;
}

void GrpcServer::wait()
{
    if (started()) {
        impl_->server->Wait();
    }
}

void GrpcServer::shutdown()
{
    if (started()) {
        impl_->server->Shutdown();
    }
}

GrpcServer buildGrpcServer(const std::string& listen_address,
                           pcl_executor_t* executor)
{
    if (!executor) {
        return {};
    }

    auto impl = std::make_unique<GrpcServer::Impl>();
    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen_address, grpc::InsecureServerCredentials());

    impl->capabilities_service = std::make_unique<grpc_transport::Capabilities_ServiceImpl>(executor);
    builder.RegisterService(impl->capabilities_service.get());

    impl->planning_requirement_service = std::make_unique<grpc_transport::Planning_Requirement_ServiceImpl>(executor);
    builder.RegisterService(impl->planning_requirement_service.get());

    impl->execution_requirement_service = std::make_unique<grpc_transport::Execution_Requirement_ServiceImpl>(executor);
    builder.RegisterService(impl->execution_requirement_service.get());

    impl->state_service = std::make_unique<grpc_transport::State_ServiceImpl>(executor);
    builder.RegisterService(impl->state_service.get());

    impl->plan_service = std::make_unique<grpc_transport::Plan_ServiceImpl>(executor);
    builder.RegisterService(impl->plan_service.get());

    impl->execution_run_service = std::make_unique<grpc_transport::Execution_Run_ServiceImpl>(executor);
    builder.RegisterService(impl->execution_run_service.get());

    impl->requirement_placement_service = std::make_unique<grpc_transport::Requirement_Placement_ServiceImpl>(executor);
    builder.RegisterService(impl->requirement_placement_service.get());

    impl->server = builder.BuildAndStart();
    if (!impl->server) {
        return {};
    }
    return GrpcServer(std::move(impl));
}

} // namespace pyramid::components::autonomy_backend::services::provided
